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

#include <algorithm>
#include <utility>

#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"

using namespace gazebo::ecs;

class gazebo::ecs::EntityComponentDatabasePrivate
{
  /// \brief instances of entities
  public: std::vector<Entity> entities;

  // TODO better storage of components
  // Map EntityId/ComponentType pair to an index in this->components
  public: std::map<std::pair<EntityId, ComponentType>, int> componentIndices;
  public: std::vector<char*> components;
  // Map EntityId/ComponentType pair to the state of a component
  public: std::map<std::pair<EntityId, ComponentType>, Difference> differences;

  /// \brief update queries because this entity's components have changed
  public: void UpdateQueries(EntityId _id);

  /// \brief check if an entity has these components
  /// \returns true iff entity has all components in the set
  public: bool EntityMatches(EntityId _id,
              const std::set<ComponentType> &_types) const;

  /// \brief Queries on this manager
  public: std::vector<EntityQuery> queries;
};

/////////////////////////////////////////////////
EntityComponentDatabase::EntityComponentDatabase()
: dataPtr(new EntityComponentDatabasePrivate)
{
}

/////////////////////////////////////////////////
EntityComponentDatabase::~EntityComponentDatabase()
{
  // Call destructor on the components
  for (auto const &kv : this->dataPtr->componentIndices)
  {
    const ComponentType &type = kv.first.second;
    const int index = kv.second;

    char *storage = this->dataPtr->components[index];
    void *data = static_cast<void*>(storage);

    ComponentTypeInfo info = ComponentFactory::TypeInfo(type);
    info.destructor(data);
    delete [] storage;
  }
}

/////////////////////////////////////////////////
std::pair<EntityQueryId, bool> EntityComponentDatabase::AddQuery(
    EntityQuery &&_query)
{
  bool isDuplicate = false;
  EntityQueryId result = -1;
  for (size_t i = 0; i < this->dataPtr->queries.size(); ++i)
  {
    if (this->dataPtr->queries[i] == _query)
    {
      result = i;
      // Already have this query, bail
      isDuplicate = true;
      break;
    }
  }

  if (!isDuplicate)
  {
    auto const types = _query.ComponentTypes();
    this->dataPtr->queries.push_back(std::move(_query));
    result = this->dataPtr->queries.size() - 1;
    auto &nonConstQuery = this->dataPtr->queries.back();
    for (int id = 0; id < this->dataPtr->entities.size(); ++id)
    {
      if (this->dataPtr->EntityMatches(id, types))
      {
        nonConstQuery.AddEntity(id);
      }
    }
  }

  return {result, !isDuplicate};
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::RemoveQuery(const EntityQueryId _id)
{
  if (_id >= 0 && _id < this->dataPtr->queries.size())
  {
    this->dataPtr->queries.erase(this->dataPtr->queries.begin() + _id);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
EntityId EntityComponentDatabase::CreateEntity()
{
  // TODO Reuse deleted entity ids
  EntityId id = this->dataPtr->entities.size();
  this->dataPtr->entities.push_back(
      std::move(gazebo::ecs::Entity(this, id)));
  return id;
}

/////////////////////////////////////////////////
gazebo::ecs::Entity &EntityComponentDatabase::Entity(EntityId _id) const
{
  if (_id >= 0 && _id < this->dataPtr->entities.size())
    return this->dataPtr->entities[_id];
  else
  {
    return EntityNull;
  }
}

/////////////////////////////////////////////////
void *EntityComponentDatabase::AddComponent(EntityId _id, ComponentType _type)
{
  void *component = nullptr;
  auto key = std::make_pair(_id, _type);
  if (this->dataPtr->componentIndices.find(key) ==
      this->dataPtr->componentIndices.end())
  {
    // Allocate memory and call constructor
    ComponentTypeInfo info = ComponentFactory::TypeInfo(_type);
    // TODO store components of same type in adjacent memory
    char *storage = new char[info.size];
    component = static_cast<void *>(storage);
    info.constructor(component);

    // TODO store metadata adjacent to component data in memory
    this->dataPtr->differences[key] = WAS_CREATED;

    auto index = this->dataPtr->components.size();
    this->dataPtr->componentIndices[key] = index;
    this->dataPtr->components.push_back(storage);
    this->dataPtr->UpdateQueries(_id);
  }

  return component;
}

/////////////////////////////////////////////////
void *EntityComponentDatabase::EntityComponent(EntityId _id,
    ComponentType _type) const
{
  void *component = nullptr;
  auto key = std::make_pair(_id, _type);
  if (this->dataPtr->componentIndices.find(key) !=
      this->dataPtr->componentIndices.end())
  {
    auto index = this->dataPtr->componentIndices[key];
    char *data = this->dataPtr->components[index];
    component = static_cast<void*>(data);
  }
  return component;
}

/////////////////////////////////////////////////
bool EntityComponentDatabasePrivate::EntityMatches(EntityId _id,
    const std::set<ComponentType> &_types) const
{
  for (auto const &type : _types)
  {
    auto key = std::make_pair(_id, type);
    if (this->componentIndices.find(key) == this->componentIndices.end())
    {
      return false;
    }
  }
  return true;
}

/////////////////////////////////////////////////
void EntityComponentDatabasePrivate::UpdateQueries(EntityId _id)
{
  for (auto &query : this->queries)
  {
    if (this->EntityMatches(_id, query.ComponentTypes()))
      query.AddEntity(_id);
  }
}

/////////////////////////////////////////////////
const EntityQuery &EntityComponentDatabase::Query(
    const EntityQueryId _index) const
{
  if (_index >= 0 && _index < this->dataPtr->queries.size())
    return this->dataPtr->queries[_index];
  return EntityQueryNull;
}

/////////////////////////////////////////////////
Difference EntityComponentDatabase::IsDifferent(EntityId _id,
    ComponentType _type) const
{
  Difference d = NO_DIFFERENCE;
  auto key = std::make_pair(_id, _type);
  auto iter = this->dataPtr->differences.find(key);
  if (iter != this->dataPtr->differences.end())
  {
    d = iter->second;
  }
  return d;
}

/////////////////////////////////////////////////
void EntityComponentDatabase::UpdateBegin()
{
  this->dataPtr->differences.clear();
}

/////////////////////////////////////////////////
void EntityComponentDatabase::MarkAsModified(EntityId _id, ComponentType _type)
{
  auto key = std::make_pair(_id, _type);
  auto iter = this->dataPtr->differences.find(key);
  if (iter == this->dataPtr->differences.end() || iter->second == NO_DIFFERENCE)
  {
    this->dataPtr->differences[key] = WAS_MODIFIED;
  }
}
