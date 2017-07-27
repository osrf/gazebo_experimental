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

#ifndef GAZEBO_SERVER_ENTITYMANAGER_HH_
#define GAZEBO_SERVER_ENTITYMANAGER_HH_

#include <memory>

#include "gazebo/server/Types.hh"
#include "gazebo/server/Entity.hh"
#include "gazebo/server/ComponentFactory.hh"

namespace gazebo
{
  namespace server
  {
    /// \brief Forward declaration
    class EntityQuery;

    /// \brief Forward declaration
    class EntityManagerPrivate;

    /// \brief Stores and retrieves entities/components efficiently
    ///
    /// This class stores entities and components, and provides efficient
    /// queries for retrieving them.
    ///
    /// This class is for internal use only.
    class EntityManager
    {
      /// \brief Constructor
      public: EntityManager();

      /// \brief Destructor
      public: ~EntityManager();

      /// \brief Creates a new entity
      /// \return A reference to the new entity
      public: Entity &CreateEntity();

      /// \brief Removes an existing entity. This will also remove the
      /// components attached to the entity.
      /// \returns true if the entity existed and was removed.
      public: bool RemoveEntity(const EntityId _id);

      /// \brief Add a query for entities. This will copy the _query object.
      /// \param[in] _query The query to add
      /// \returns The index of the query. If the query was not added, then
      /// the result will be -1.
      public: EntityQueryId AddQuery(EntityQuery &&_query);

      /// \brief Remove a query for entities
      /// \param[in] _id Id of the query to remove.
      /// \returns True if query was successfully removed
      public: bool RemoveQuery(const EntityQueryId _id);

      /// \brief Get a query based on an index.
      /// \param[in] _index Index of the query.
      /// \return The EntityQuery, or NullEntityQuery on error.
      public: const EntityQuery &Query(const EntityQueryId _index) const;

      /// \brief Get an Entity instance by Id
      /// \param[in] _id Id of the Entity to retrieve.
      /// \return Reference to the Entity
      public: ::gazebo::server::Entity &EntityById(const EntityId _id) const;

      /// \brief Update the queries associated with an entity.
      /// \param[in] _id Entity to update
      public: void UpdateQueries(EntityId _id);

      /// \brief Update all queries for all entities
      public: void UpdateQueries();

      /// \brief Private IMPLementation pointer
      private: std::unique_ptr<EntityManagerPrivate> dataPtr;
    };
  }
}

#endif
