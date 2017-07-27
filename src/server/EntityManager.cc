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
#include <assert.h>
#include <set>
#include <utility>
#include <ignition/common/Console.hh>

#include "gazebo/server/EntityManager.hh"
#include "gazebo/server/EntityQuery.hh"

using namespace gazebo::server;

class gazebo::server::EntityManagerPrivate
{
  /// \brief update queries because this entity's components have changed
  public: void UpdateQueries(EntityId _id);

  /// \brief return true iff the entity exists
  public: bool EntityExists(EntityId _id) const;

  /// \brief instances of entities
  public: std::vector<Entity> entities;

  /// \brief deleted entity ids that can be reused
  public: std::set<EntityId> freeIds;

  /// \brief Queries on this manager
  public: std::vector<EntityQuery> queries;

  /// \brief mutex used for thread safety
  /// \todo thread safety without locking everything
  public: std::mutex mutex;
};

/////////////////////////////////////////////////
EntityManager::EntityManager()
: dataPtr(new EntityManagerPrivate)
{
}

/////////////////////////////////////////////////
EntityManager::~EntityManager()
{
}

/////////////////////////////////////////////////
Entity &EntityManager::CreateEntity()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  EntityId id;
  if (!this->dataPtr->freeIds.empty())
  {
    // Reuse the smallest deleted EntityId
    id = *(this->dataPtr->freeIds.begin());
    this->dataPtr->freeIds.erase(this->dataPtr->freeIds.begin());
    this->dataPtr->entities[id] = std::move(gazebo::server::Entity(id));
  }
  else
  {
    // Create a brand new Id
    id = this->dataPtr->entities.size();
    this->dataPtr->entities.emplace_back(id);
  }

  return this->dataPtr->entities[id];
}

/////////////////////////////////////////////////
bool EntityManager::RemoveEntity(const EntityId _id)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  bool success = false;

  if (this->dataPtr->EntityExists(_id))
  {
    // Add the entity to the list of freeIds.
    this->dataPtr->freeIds.insert(_id);

    // Remove the components attached to the entity.
    if (!ComponentFactory::RemoveComponents(_id))
    {
      ignerr << "Unable to remove components from Entity[" << _id << "]\n";
      success = false;
    }
    else
      success = true;
  }
  return success;
}

/////////////////////////////////////////////////
EntityQueryId EntityManager::AddQuery(EntityQuery &&_query)
{
  EntityQueryId result = -1;

  // Find an existing entity query
  auto it = std::find_if(this->dataPtr->queries.begin(),
      this->dataPtr->queries.end(), [&_query](const EntityQuery &q)
      {
        return q == _query;
      });

  // If the query did not exist, then create a new one
  if (it == this->dataPtr->queries.end())
  {
    // Copy the query
    this->dataPtr->queries.emplace_back(std::move(_query));
    auto &insertedQuery = this->dataPtr->queries.back();

    insertedQuery.SetEntityCb(std::bind(
          &EntityManager::EntityById, this, std::placeholders::_1));

    // ID of the query
    result = this->dataPtr->queries.size() - 1;

    auto const &types = insertedQuery.ComponentTypes();

    for (auto const &ent : this->dataPtr->entities)
    {
      // Check that entity exists and has the required components
      if (ComponentFactory::EntityMatches(ent.Id(), types))
        insertedQuery.AddEntity(ent.Id());
    }
  }

  // An EntityQueryId is the position of the query in the dataptr->queries
  // vector.
  return result;
}

/////////////////////////////////////////////////
bool EntityManager::RemoveQuery(const EntityQueryId _id)
{
  if (_id >= 0 && _id < this->dataPtr->queries.size())
  {
    this->dataPtr->queries.erase(this->dataPtr->queries.begin() + _id);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
const EntityQuery &EntityManager::Query(
    const EntityQueryId _index) const
{
  if (_index >= 0 && _index < this->dataPtr->queries.size())
    return this->dataPtr->queries[_index];

  return NullEntityQuery;
}

/////////////////////////////////////////////////
gazebo::server::Entity &EntityManager::EntityById(
    const EntityId _id) const
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->EntityExists(_id))
    return this->dataPtr->entities[_id];
  else
    return NullEntity;
}

/////////////////////////////////////////////////
bool EntityManagerPrivate::EntityExists(EntityId _id) const
{
  // True if the vector is big enough to have used this id
  bool isWithinRange = _id >= 0 && _id < this->entities.size();
  bool isNotDeleted = this->freeIds.find(_id) == this->freeIds.end();
  return isWithinRange && isNotDeleted;
}

/////////////////////////////////////////////////
void EntityManager::UpdateQueries()
{
  for (auto const &ent : this->dataPtr->entities)
  {
    this->UpdateQueries(ent.Id());
  }
}

/////////////////////////////////////////////////
void EntityManager::UpdateQueries(EntityId _id)
{
  for (auto &query : this->dataPtr->queries)
  {
    if (ComponentFactory::EntityMatches(_id, query.ComponentTypes()))
      query.AddEntity(_id);
    else if (query.HasEntity(_id))
      query.RemoveEntity(_id);
  }
}
