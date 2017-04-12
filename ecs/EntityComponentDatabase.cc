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
#include <set>
#include <utility>

#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"


using namespace gazebo::ecs;

class gazebo::ecs::EntityComponentDatabasePrivate
{
  /// \brief instances of entities
  public: std::vector<Entity> entities;

  /// \brief deleted entity ids
  public: std::set<EntityId> freeIds;

  // TODO better storage of components
  // Map EntityId/ComponentType pair to an index in this->components
  public: std::map<std::pair<EntityId, ComponentType>, int> componentIndices;
  public: std::vector<char*> components;

  /// \brief update queries because this entity's components have changed
  public: void UpdateQueries(EntityId _id);

  /// \brief return true iff the entity exists
  public: bool EntityExists(EntityId _id) const;

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
  EntityId id;
  if (!this->dataPtr->freeIds.empty())
  {
    // Reuse the smallest deleted EntityId
    id = *(this->dataPtr->freeIds.begin());
    this->dataPtr->freeIds.erase(this->dataPtr->freeIds.begin());
    this->dataPtr->entities[id] = std::move(gazebo::ecs::Entity(this, id));
  }
  else
  {
    // Create a brand new Id
    id = this->dataPtr->entities.size();
    this->dataPtr->entities.push_back(
        std::move(gazebo::ecs::Entity(this, id)));
  }
  return id;
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::DeleteEntity(EntityId _id)
{
  bool success = false;
  if (this->dataPtr->EntityExists(_id))
  {
    std::vector<ComponentType> knownTypes = ComponentFactory::Types();
    for (const ComponentType type : knownTypes)
    {
      this->RemoveComponent(_id, type);
    }
    this->dataPtr->freeIds.insert(_id);
    success = true;
  }
  return success;
}


/////////////////////////////////////////////////
gazebo::ecs::Entity &EntityComponentDatabase::Entity(EntityId _id) const
{
  if (this->dataPtr->EntityExists(_id))
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

    auto index = this->dataPtr->components.size();
    this->dataPtr->componentIndices[key] = index;
    this->dataPtr->components.push_back(storage);
    this->dataPtr->UpdateQueries(_id);
  }

  return component;
}

/////////////////////////////////////////////////
bool EntityComponentDatabase::RemoveComponent(EntityId _id, ComponentType _type)
{
  bool success = false;
  auto key = std::make_pair(_id, _type);
  auto kvIter = this->dataPtr->componentIndices.find(key);
  if (kvIter != this->dataPtr->componentIndices.end())
  {
    auto index = kvIter->second;
    // call destructor
    void *component = nullptr;
    ComponentTypeInfo info = ComponentFactory::TypeInfo(_type);
    char *storage = this->dataPtr->components[index];
    component = static_cast<void *>(storage);
    info.destructor(component);

    // TODO don't deallocate, need smarter storage
    delete [] storage;
    this->dataPtr->components.erase(this->dataPtr->components.begin() + index);
    this->dataPtr->componentIndices.erase(kvIter);
    // Update the indexes beyond
    for (auto &otherKvIter : this->dataPtr->componentIndices)
    {
      int &otherIndex = otherKvIter.second;
      if (otherIndex > index)
        --otherIndex;
    }

    this->dataPtr->UpdateQueries(_id);
    success = true;
  }

  return success;
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
bool EntityComponentDatabasePrivate::EntityExists(EntityId _id) const
{
  // True if the vector is big enough to have used this id
  bool isWithinRange = _id >= 0 && _id < this->entities.size();
  bool isNotDeleted = this->freeIds.find(_id) == this->freeIds.end();
  return isWithinRange && isNotDeleted;
}

/////////////////////////////////////////////////
const EntityQuery &EntityComponentDatabase::Query(
    const EntityQueryId _index) const
{
  if (_index >= 0 && _index < this->dataPtr->queries.size())
    return this->dataPtr->queries[_index];
  return EntityQueryNull;
}
