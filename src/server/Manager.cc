/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * *     http://www.apache.org/licenses/LICENSE-2.0 * * Unless required by applicable law or agreed to in writing, software
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

#include <ignition/math/graph.hh>

#include <sdf/sdf.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/WorkerPool.hh>
#include <gazebo/Config.hh>

#include "gazebo/server/components/Pose.hh"

#include "gazebo/server/Entity.hh"
#include "gazebo/server/EntityManager.hh"
#include "gazebo/server/EntityQuery.hh"

#include "gazebo/server/QueryRegistrar.hh"
#include "gazebo/util/DiagnosticsManager.hh"

#include "gazebo/server/Manager.hh"

using namespace gazebo::server;

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
class gazebo::server::ManagerPrivate
{
  public: Manager *mgr = nullptr;

  public: ignition::math::graph::UndirectedGraph<EntityId, int> graph;

  public: std::atomic<bool> stop;

  public: std::unique_ptr<std::thread> runThread;

  /// \brief Systems that are added to the manager
  public: std::vector<std::unique_ptr<System> > systems;

  /// \brief System info associated with a system by index
  public: std::vector<SystemInfo> systemInfo;

  /// \brief Pool of workers to do stuff in parallel
  public: ignition::common::WorkerPool workerThreads;

  /// \brief Handles storage and quering of components
  public: EntityManager entityMgr;

  /// \brief Holds the current simulation time
  public: ignition::common::Time simTime;

  /// \brief Holds the next simulation time
  public: ignition::common::Time nextSimTime;

  /// \brief true if the simulation is paused
  public: bool paused = false;

  /// \brief tool for publishing diagnostic info
  public: util::DiagnosticsManager diagnostics;

  /// \brief Updates the state and systems once
  public: void UpdateOnce();

  /// \brief Create components using SDF
  public: void CreateComponents(sdf::SDFPtr _sdf);

  /// \brief Helper function that recursively reads an SDF structure
  public: void CreateComponentRecurse(sdf::ElementPtr _elem,
    ignition::math::graph::UndirectedGraph<EntityId, int> &_graph,
    ignition::math::graph::VertexId _parent);
};

/////////////////////////////////////////////////
Manager::Manager()
: dataPtr(new ManagerPrivate)
{
  this->dataPtr->mgr = this;
  this->dataPtr->diagnostics.Init("server/Manager");

  /// \brief configure paths and callbacks for sdformat model lookups
  std::string modelPath;
  if (!ignition::common::env("GAZEBO_MODEL_PATH", modelPath))
  {
    sdf::addURIPath("model://", modelPath);
  }

  std::string homePath;
  if (!ignition::common::env("HOME", homePath))
  {
    sdf::addURIPath("model://", homePath + "/.gazebo/models");
  }

  sdf::setFindCallback([] (const std::string &) -> std::string {return "";});
}

/////////////////////////////////////////////////
Manager::~Manager()
{
}

/////////////////////////////////////////////////
bool Manager::Load(const std::string &_worldFile)
{
  // Load the systems we need
  this->LoadSystems({"physics"});

  bool success = true;
  ignition::common::SystemPaths sp;

  std::string fullPath = sp.LocateLocalFile(_worldFile,
      {"", "./", GAZEBO_WORLD_INSTALL_DIR});

  if (fullPath.empty())
  {
    ignwarn << "Cannot find [" << _worldFile << "]" << std::endl;
    success = false;
  }
  else
  {
    igndbg << "Loading world [" << fullPath << "]" << std::endl;

    sdf::SDFPtr sdfWorld(new sdf::SDF());
    sdf::init(sdfWorld);

    if (sdf::readFile(fullPath, sdfWorld))
    {
      success = true;
      this->dataPtr->CreateComponents(sdfWorld);
    }
  }

  return success;
}

/////////////////////////////////////////////////
bool Manager::LoadSystem(const std::string &_name)
{
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  std::string pluginPath;

  if (!ignition::common::env("GAZEBO_SYSTEM_PLUGIN_PATH", pluginPath))
  {
    pluginPath = GAZEBO_SYSTEM_PLUGIN_PATH;
  }
  sp.AddPluginPaths(pluginPath);

  bool success = false;
  std::string pathToLibrary = sp.FindSharedLibrary(_name);
  std::string pluginName = pluginLoader.LoadLibrary(pathToLibrary);

  if (!pluginName.empty())
  {
    std::unique_ptr<gazebo::server::System> sys(
      pluginLoader.Instantiate<gazebo::server::System>(pluginName));

    if (sys)
    {
      SystemInfo sysInfo;
      sysInfo.name = _name;

      EntityQueryRegistrar registrar;
      sys->Init(registrar);
      for (auto &registration : registrar.Registrations())
      {
        auto result = this->dataPtr->entityMgr.AddQuery(
            std::move(registration.first));

        if (result >= 0)
        {
          sysInfo.updates.push_back(
              std::make_pair(result, registration.second));
        }
        else
        {
          ignerr << "Unable to add EntityQuery\n";
        }
      }

      this->dataPtr->systems.push_back(std::move(sys));
      this->dataPtr->systemInfo.push_back(sysInfo);
      success = true;
    }
  }

  return success;
}

/////////////////////////////////////////////////
bool Manager::LoadSystems(const std::vector<std::string> &_libs)
{
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");

  for (auto const &libName : _libs)
  {
    if (!this->LoadSystem(libName))
    {
      ignerr << "Failed to load " << libName << std::endl;
      return false;
    }
    else
    {
      igndbg << "Loaded plugin " << libName << std::endl;
    }
  }

  return true;
}

/////////////////////////////////////////////////
void Manager::Run()
{
  this->dataPtr->runThread.reset(new std::thread(&Manager::RunImpl, this));
}

/////////////////////////////////////////////////
void Manager::Stop()
{
  this->dataPtr->stop = true;
  if (this->dataPtr->runThread)
    this->dataPtr->runThread->join();
}

/////////////////////////////////////////////////
void Manager::RunImpl()
{
  const double realTimeFactor = 1.0;
  while (!this->dataPtr->stop)
  {
    this->UpdateOnce(realTimeFactor);
  }
}

/////////////////////////////////////////////////
void Manager::UpdateOnce()
{
  this->dataPtr->diagnostics.UpdateBegin(this->dataPtr->simTime);
  this->dataPtr->UpdateOnce();
  this->dataPtr->diagnostics.UpdateEnd();
}

/////////////////////////////////////////////////
void Manager::UpdateOnce(const double _realTimeFactor)
{
  this->dataPtr->diagnostics.UpdateBegin(this->dataPtr->simTime);

  ignition::common::Time startWallTime = ignition::common::Time::SystemTime();
  ignition::common::Time startSimTime = this->dataPtr->simTime;

  this->dataPtr->UpdateOnce();

  ignition::common::Time endSimTime = this->dataPtr->simTime;
  ignition::common::Time endWallTime = ignition::common::Time::SystemTime();

  this->dataPtr->diagnostics.StartTimer("sleep");
  const ignition::common::Time scalar(_realTimeFactor);
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
  // Update systems in parallel
  // \todo: This should be replaced with an execution model. For example, we
  // probably want to run physics first before other systems.
  for(auto const &sysInfo : this->systemInfo)
  {
    this->workerThreads.AddWork([&sysInfo, this] ()
      {
        this->diagnostics.StartTimer(sysInfo.name);
        for (auto &updateInfo : sysInfo.updates)
        {
          updateInfo.second(this->mgr, this->entityMgr.Query(updateInfo.first));
        }
        this->diagnostics.StopTimer(sysInfo.name);
      });
  }

  // Wait for results.
  if (!this->workerThreads.WaitForResults())
    ignerr << "One or more worker threads failed to return\n";

  // Advance sim time according to what was set last update
  this->simTime = this->nextSimTime;
}

//////////////////////////////////////////////////
void ManagerPrivate::CreateComponents(sdf::SDFPtr _sdf)
{
  // Create the component and populate the graph
  this->CreateComponentRecurse(_sdf->Root(), this->graph,
      ignition::math::graph::kNullId);

  // Make sure the queries are all up to date
  this->entityMgr.UpdateQueries();
}

//////////////////////////////////////////////////
void ManagerPrivate::CreateComponentRecurse(sdf::ElementPtr _elem,
    ignition::math::graph::UndirectedGraph<EntityId, int> &_graph,
    ignition::math::graph::VertexId _parent)
{
  std::string type = _elem->GetName();

  ignition::math::graph::VertexId v = ignition::math::graph::kNullId;

  // Special case for world, model, link, etc.
  // \todo: A better solution than hardcoding strings would be nice.
  if (type == "world" || type == "model" || type == "link" ||
      type == "collision" || type == "visual")
  {
    // Create the entity
    Entity &entity = this->entityMgr.CreateEntity();

    // Add a vertex to the scene graph.
    v = _graph.AddVertex(_elem->GetName(), entity.Id()).Id();

    // Add an edge to the scene graph.
    _graph.AddEdge({_parent, v}, 0);
  }
  else
  {
    // Add a component to the entity
    if (!this->entityMgr.CreateComponent(_graph.VertexFromId(_parent).Data(),
          _elem))
    {
      ignwarn << "Unable to create a component of type[" << _elem->GetName()
        << "] to entity with Id[" << _graph.VertexFromId(_parent).Data()
        << "]\n";
    }
  }

  // Read the other elements in the SDF file.
  sdf::ElementPtr child = _elem->GetFirstElement();
  while (child)
  {
    this->CreateComponentRecurse(child, _graph,
        v == ignition::math::graph::kNullId ? _parent : v);
    child = child->GetNextElement();
  }
}

/////////////////////////////////////////////////
const ignition::common::Time &Manager::SimulationTime() const
{
  return this->dataPtr->simTime;
}

/////////////////////////////////////////////////
bool Manager::SetSimulationTime(const ignition::common::Time &_newTime)
{
  if (!this->Paused())
  {
    this->dataPtr->nextSimTime = _newTime;
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
void Manager::SetPaused(const bool _pause)
{
  this->dataPtr->paused = _pause;
}

/////////////////////////////////////////////////
bool Manager::Paused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
Entity &Manager::CreateEntity(const std::string &_name)
{
  auto &entity = this->dataPtr->entityMgr.CreateEntity();

  // Add a vertex to the scene graph.
  this->dataPtr->graph.AddVertex(_name, entity.Id());

  return entity;
}

/////////////////////////////////////////////////
gazebo::server::Entity &Manager::EntityById(
    const EntityId _id) const
{
  return this->dataPtr->entityMgr.EntityById(_id);
}

/////////////////////////////////////////////////
ignition::math::graph::UndirectedGraph<EntityId, int> &Manager::Graph() const
{
  return this->dataPtr->graph;
}
