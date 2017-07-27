/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <atomic>
#include <queue>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/common/Console.hh>
#include <sdf/sdf.hh>
#include <ignition/common/WorkerPool.hh>

#include "gazebo/ecs/Componentizer.hh"
#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/QueryRegistrar.hh"
#include "gazebo/util/DiagnosticsManager.hh"

using namespace gazebo;
using namespace ecs;

/// \brief Forward declaration
class System;

/////////////////////////////////////////////////
/// \brief struct to hold information required for updating a system
struct SystemInfo
{
  /// \brief name of a system
  public: std::string name;

  /// \brief List of callbacks to call
  public: std::vector<std::pair<EntityQueryId, QueryCallback> > updates;
};

/////////////////////////////////////////////////
class gazebo::ecs::ManagerPrivate
{
  /// \brief Componentizers that are added to the manager
  public: std::vector<std::unique_ptr<Componentizer> > componentizers;

  /// \brief Systems that are added to the manager
  public: std::vector<std::unique_ptr<System> > systems;

  /// \brief System info associated with a system by index
  public: std::vector<SystemInfo> systemInfo;

  /// \brief Pool of workers to do stuff in parallel
  public: ignition::common::WorkerPool workerThreads;

  /// \brief Handles storage and quering of components
  public: EntityComponentDatabase database;

  /// \brief Holds the current simulation time
  public: ignition::common::Time simTime;

  /// \brief Holds the current time since the simulation started.
  public: ignition::common::Time realTime;

  /// \brief Holds the system time when simulation started.
  public: ignition::common::Time realTimeStart;

  /// \brief Holds the next simulation time
  public: ignition::common::Time nextSimTime;

  /// \brief count of how many things want simulation time paused
  public: std::atomic<int> pauseCount;

  /// \brief true if the simulation is paused
  public: bool paused = false;

  /// \brief tool for publishing diagnostic info
  public: util::DiagnosticsManager diagnostics;

  /// \brief mutex for protecting diagnostics when doing multi-threaded stuff
  public: std::mutex diagMtx;

  /// \brief Updates the state and systems once
  public: void UpdateOnce();

  /// \brief Invokes componentizers on SDF
  public: void Componentize(Manager *_mgr, sdf::SDF &_sdf);
};

/////////////////////////////////////////////////
Manager::Manager()
: dataPtr(new ManagerPrivate)
{
  this->dataPtr->pauseCount = 0;
  this->dataPtr->diagnostics.Init("ecs:Manager");
}

/////////////////////////////////////////////////
Manager::~Manager()
{
}

/////////////////////////////////////////////////
EntityId Manager::CreateEntity()
{
  return this->dataPtr->database.CreateEntity();
}

/////////////////////////////////////////////////
bool Manager::DeleteEntity(EntityId _id)
{
  return this->dataPtr->database.DeleteEntity(_id);
}

/////////////////////////////////////////////////
void Manager::UpdateOnce()
{
  this->dataPtr->diagnostics.UpdateBegin(this->dataPtr->simTime);
  this->dataPtr->UpdateOnce();
  this->dataPtr->diagnostics.UpdateEnd();
}

/////////////////////////////////////////////////
void Manager::UpdateOnce(double _real_time_factor)
{
  this->dataPtr->diagnostics.UpdateBegin(this->dataPtr->simTime);

  ignition::common::Time startWallTime = ignition::common::Time::SystemTime();
  ignition::common::Time startSimTime = this->dataPtr->simTime;

  this->dataPtr->UpdateOnce();

  ignition::common::Time endSimTime = this->dataPtr->simTime;
  ignition::common::Time endWallTime = ignition::common::Time::SystemTime();

  this->dataPtr->diagnostics.StartTimer("sleep");
  const ignition::common::Time scalar(_real_time_factor);
  const ignition::common::Time deltaWall = endWallTime - startWallTime;
  const ignition::common::Time deltaSim = endSimTime - startSimTime;
  const ignition::common::Time expectedDeltaWall = deltaSim / scalar;
  if (deltaWall < expectedDeltaWall)
  {
    ignition::common::Time sleep = (expectedDeltaWall - deltaWall);
    std::this_thread::sleep_for(std::chrono::seconds(sleep.sec) +
        std::chrono::nanoseconds(sleep.nsec));
  }
  this->dataPtr->diagnostics.StopTimer("sleep");

  this->dataPtr->diagnostics.UpdateEnd();
}

/////////////////////////////////////////////////
void ManagerPrivate::UpdateOnce()
{
  // Decide at the beginning of every update if sim time is paused or not.
  // Some systems (like rendering a camera for a GUI) need to continue to run
  // even when simulation time is paused, so it's up to each system to check
  this->paused = this->pauseCount;

  // Let database do some stuff before starting the new update
  this->diagnostics.StartTimer("database");
  this->database.Update();
  this->diagnostics.StopTimer("database");

  // Update systems in parallel
  for(auto &sysInfo : this->systemInfo)
  {
    this->workerThreads.AddWork([&sysInfo, this] ()
        {
          {
            std::unique_lock<std::mutex> diagLock(this->diagMtx);
            this->diagnostics.StartTimer(sysInfo.name);
          }
          for (auto &updateInfo : sysInfo.updates)
          {
            auto query = this->database.Query(updateInfo.first);
            QueryCallback cb = updateInfo.second;
            cb(query);
          }
          {
             std::unique_lock<std::mutex> diagLock(this->diagMtx);
             this->diagnostics.StopTimer(sysInfo.name);
          }
        });
  }
  assert(this->workerThreads.WaitForResults());

  // Advance sim time according to what was set last update
  this->simTime = this->nextSimTime;

  // Update real time
  if (this->realTimeStart == ignition::common::Time::Zero)
    this->realTimeStart = ignition::common::Time::SystemTime();
  this->realTime = ignition::common::Time::SystemTime() - this->realTimeStart;
}

/////////////////////////////////////////////////
bool Manager::LoadSystem(const std::string &_name,
    std::unique_ptr<System> _sys)
{
  bool success = false;
  if (_sys)
  {
    SystemInfo sysInfo;
    sysInfo.name = _name;
    QueryRegistrar registrar;
    _sys->Manager(this);
    _sys->Init(registrar);
    for (auto registration : registrar.Registrations())
    {
      EntityQuery &query = registration.first;
      QueryCallback &cb = registration.second;
      auto result = this->dataPtr->database.AddQuery(query);
      sysInfo.updates.push_back(std::make_pair(result.first, cb));
    }
    this->dataPtr->systems.push_back(std::move(_sys));
    this->dataPtr->systemInfo.push_back(sysInfo);
    success = true;
  }
  return success;
}

//////////////////////////////////////////////////
bool Manager::LoadComponentizer(std::unique_ptr<Componentizer> _cz)
{
  bool success = false;
  if (_cz)
  {
    _cz->Init();
    this->dataPtr->componentizers.push_back(std::move(_cz));
    success = true;
  }
  return success;
}

//////////////////////////////////////////////////
void ManagerPrivate::Componentize(Manager *_mgr, sdf::SDF &_sdf)
{
  std::unordered_map<sdf::Element*, EntityId> ids;
  // breadth-first componentization
  std::queue<sdf::ElementPtr> elementQueue;
  elementQueue.push(_sdf.Root());
  while (!elementQueue.empty())
  {
    sdf::ElementPtr nextElement = elementQueue.front();
    elementQueue.pop();

    assert(ids.find(nextElement.get()) == ids.end());

    // An entity makes it easier to group components from different
    // componentizers. However, they are free to create their own entities
    EntityId groupId = this->database.CreateEntity();
    ids[nextElement.get()] = groupId;

    for (auto &cz : this->componentizers)
    {
      cz->FromSDF(*_mgr, *nextElement, ids);
    }

    sdf::ElementPtr child = nextElement->GetFirstElement();
    while (child)
    {
      elementQueue.push(child);
      child = child->GetNextElement();
    }
  }
}

//////////////////////////////////////////////////
bool Manager::LoadWorldFromPath(const std::string &_path)
{
  bool success = false;
  sdf::SDFPtr sdfWorld(new sdf::SDF());
  sdf::init(sdfWorld);

  if (sdf::readFile(_path, sdfWorld))
  {
    success = true;
    this->dataPtr->Componentize(this, *sdfWorld);
  }

  return success;
}

/////////////////////////////////////////////////
bool Manager::LoadWorldFromSDFString(const std::string &_world)
{
  bool success = false;
  sdf::SDFPtr sdfWorld(new sdf::SDF());
  sdf::init(sdfWorld);

  if (sdf::readString(_world, sdfWorld))
  {
    success = true;
    this->dataPtr->Componentize(this, *sdfWorld);
  }

  return success;
}

/////////////////////////////////////////////////
gazebo::ecs::Entity &Manager::Entity(const EntityId _id) const
{
  return this->dataPtr->database.Entity(_id);
}

/////////////////////////////////////////////////
const ignition::common::Time &Manager::RealTime() const
{
  return this->dataPtr->realTime;
}

/////////////////////////////////////////////////
const ignition::common::Time &Manager::SimulationTime() const
{
  return this->dataPtr->simTime;
}

/////////////////////////////////////////////////
bool Manager::SimulationTime(const ignition::common::Time &_newTime)
{
  if (!this->Paused())
  {
    this->dataPtr->nextSimTime = _newTime;
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
int Manager::BeginPause()
{
  ignmsg << "Manager: begin pause" << std::endl;
  return ++(this->dataPtr->pauseCount);
}

/////////////////////////////////////////////////
int Manager::EndPause()
{
  ignmsg << "Manager: end pause" << std::endl;
  int currentCount = this->dataPtr->pauseCount;
  while (currentCount && !this->dataPtr->pauseCount.compare_exchange_weak(
        currentCount, currentCount - 1))
  {
    // Intentionally do nothing, compare_exchange_weak modifies currentCount
  }
  return currentCount > 0 ? currentCount - 1 : currentCount;
}

/////////////////////////////////////////////////
bool Manager::Paused() const
{
  return this->dataPtr->paused;
}

//////////////////////////////////////////////////
std::set<gazebo::ecs::EntityId> Manager::QueryEntities(
            const std::vector<std::string> &_components)
{
  // Make a query from the component names
  // demand the database to give us the results now
  EntityQuery q;
  for (const std::string compName : _components)
  {
    q.AddComponent(compName);
  }

  this->dataPtr->database.InstantQuery(q);
  return q.EntityIds();
}
