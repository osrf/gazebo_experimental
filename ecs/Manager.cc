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
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"

using namespace gazebo;
using namespace ecs;

/// \brief Forward declaration
class System;

class gazebo::ecs::ManagerPrivate
{
  /// \brief Systems that are added to the manager
  public: std::vector<
          std::pair<std::unique_ptr<System>, EntityQueryId>> systems;

  /// \brief Handles storage and quering of components
  public: EntityComponentDatabase database;
};

/////////////////////////////////////////////////
Manager::Manager()
: dataPtr(new ManagerPrivate)
{
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
bool Manager::AddQuery(EntityQuery &&_query)
{
  return this->dataPtr->database.AddQuery(std::move(_query)).second;
}

/////////////////////////////////////////////////
void Manager::UpdateSystems(const double _dt)
{
  // Let database do some stuff before starting the new update
  this->dataPtr->database.UpdateBegin();

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
  for(auto &system : this->dataPtr->systems)
  {
    system.first->Update(_dt, this->dataPtr->database.Query(system.second),
        *this);
  }
}

/////////////////////////////////////////////////
bool Manager::LoadSystem(std::unique_ptr<System> _sys)
{
  bool success = false;
  if (_sys)
  {
    EntityQuery query = _sys->Init();
    auto result = this->dataPtr->database.AddQuery(std::move(query));
    this->dataPtr->systems.push_back(
        std::make_pair(std::move(_sys), result.first));
    success = true;
  }
  return success;
}

/////////////////////////////////////////////////
void *Manager::AddComponent(ComponentType _type, EntityId _id)
{
  return this->dataPtr->database.AddComponent(_id, _type);
}

/////////////////////////////////////////////////
gazebo::ecs::Entity &Manager::Entity(const EntityId _id) const
{
  return this->dataPtr->database.Entity(_id);
}
