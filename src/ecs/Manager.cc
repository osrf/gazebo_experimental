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

#include "gazebo/ecs/Componentizer.hh"
#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/QueryRegistrar.hh"

using namespace gazebo;
using namespace ecs;

/// \brief Forward declaration
class System;

/////////////////////////////////////////////////
/// \brief struct to hold information required for updating a system
struct SystemInfo
{
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

  /// \brief Handles storage and quering of components
  public: EntityComponentDatabase database;

  /// \brief Holds the current simulation time
  public: ignition::common::Time simTime;

  /// \brief Holds the next simulation time
  public: ignition::common::Time nextSimTime;

  /// \brief count of how many things want simulation time paused
  public: std::atomic<int> pauseCount;

  /// \brief true if the simulation is paused
  public: bool paused = false;
};

/////////////////////////////////////////////////
Manager::Manager()
: dataPtr(new ManagerPrivate)
{
  this->dataPtr->pauseCount = 0;
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
void Manager::UpdateSystems()
{
  // Decide at the beginning of every update if sim time is paused or not.
  // Some systems (like rendering a camera for a GUI) need to continue to run
  // even when simulation time is paused, so it's up to each system to check
  this->dataPtr->paused = this->dataPtr->pauseCount;

  // Let database do some stuff before starting the new update
  this->dataPtr->database.Update();

  // TODO There is a lot of opportunity for parallelization here
  // In general systems are run sequentially, one after the other
  //  Different Systems can run in parallel if they don't share components
  //  How to handle systems which add or remove entities?
  //  Defer creation and deletion?
  // Some systems could be run on multiple cores, such that several
  //  instances of a system each run on a subset of the entities returned
  //  in a query result.
  // Heck, entity and component data can be shared over the network to
  //  other SystemManagers to use multiple machines for one simulation

  // But this is a prototype, so here's the basic implementation
  for(auto &sysInfo : this->dataPtr->systemInfo)
  {
    for (auto &updateInfo : sysInfo.updates)
    {
      auto query = this->dataPtr->database.Query(updateInfo.first);
      QueryCallback cb = updateInfo.second;
      cb(query);
    }
  }

  // Advance sim time according to what was set last update
  this->dataPtr->simTime = this->dataPtr->nextSimTime;
}

/////////////////////////////////////////////////
bool Manager::LoadSystem(std::unique_ptr<System> _sys)
{
  bool success = false;
  if (_sys)
  {
    SystemInfo sysInfo;
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

/////////////////////////////////////////////////
bool Manager::LoadWorld(const std::string &_world)
{
  bool success = false;
  sdf::SDF sdfWorld;
  sdfWorld.SetFromString(_world);
  std::unordered_map<sdf::Element*, EntityId> ids;

  // breadth-first componentization
  std::queue<sdf::ElementPtr> elementQueue;
  elementQueue.push(sdfWorld.Root());
  while (nullptr != elementQueue.front())
  {
    sdf::ElementPtr nextElement = elementQueue.front();
    elementQueue.pop();

    assert(ids.find(nextElement.get()) == ids.end());

    // An entity makes it easier to group components from different
    // componentizers. However, they are free to create their own entities
    EntityId groupId = this->CreateEntity();
    ids[nextElement.get()] = groupId;

    igndbg << "Begin cz" << std::endl;
    for (auto &cz : this->dataPtr->componentizers)
    {
      cz->FromSDF(*this, *nextElement, ids);
    }
    igndbg << "end cz" << std::endl;

    // TODO SDFormat API for walking sdf tree
    // sdf::ElementPtr child = nextElement->GetFirstElement();
    // while (child)
    // {
    //   igndbg << "Begin children" << std::endl;
    //   elementQueue.push(child);
    //   child = nextElement->GetNextElement();
    //   igndbg << "end children" << std::endl;
    // }
  }

  return success;
}

/////////////////////////////////////////////////
gazebo::ecs::Entity &Manager::Entity(const EntityId _id) const
{
  return this->dataPtr->database.Entity(_id);
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
  return ++(this->dataPtr->pauseCount);
}

/////////////////////////////////////////////////
int Manager::EndPause()
{
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
