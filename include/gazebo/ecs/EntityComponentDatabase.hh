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

#ifndef GAZEBO_ECS_ENTITYCOMPONENTDATABASE_HH_
#define GAZEBO_ECS_ENTITYCOMPONENTDATABASE_HH_

#include <memory>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/Component.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class EntityQuery;

    /// \brief Forward declaration
    class EntityComponentDatabasePrivate;

    /// \brief Id of an EntityQuery
    using EntityQueryId = int64_t;

    /// \brief Stores and retrieves entities/components efficiently
    ///
    /// This class stores entities and components, and provides efficient
    /// queries for retrieving them.
    class EntityComponentDatabase
    {
      /// \brief Constructor
      public: EntityComponentDatabase();

      /// \brief Destructor
      public: ~EntityComponentDatabase();

      /// \brief Add a query for entities
      /// \param[in] _query The query to add
      /// \returns The index of the query and boolean in a pair. The boolean
      /// is true if query was added, false if the query already existed.
      public: std::pair<EntityQueryId, bool> AddQuery(const EntityQuery &_query);

      /// \brief Get a query based on an index.
      /// \param[in] _index Index of the query.
      /// \return The EntityQuery, or NULL_QUERY on error.
      public: const EntityQuery &Query(const EntityQueryId _index) const;

      /// \brief Remove a query for entities
      /// \param[in,out] _id ID of the query to remove.
      /// \returns True if query was successfully removed
      public: bool RemoveQuery(const EntityQueryId _id);

      /// \brief Creates a new entity
      /// \brief returns an id for the entity, or NO_ENTITY on failure
      public: EntityId CreateEntity();

      /// \brief Deletes an existing entity
      /// \returns true iff the entity existed
      public: bool DeleteEntity(EntityId _id);

      /// \brief Database clears changed components
      public: void Update();

      /// \brief Get an Entity instance by Id
      public: ::gazebo::ecs::Entity &Entity(EntityId _id) const;

      /// \brief get component type by component name
      public: gazebo::ecs::ComponentType ComponentType(const
                  std::string &name) const;

      /// \brief Add a ComponentFactory.
      /// \description Calling this method enables the database to store a new
      ///              type of component.
      /// \returns true if the factory was successfully loaded
      public: bool AddComponentFactory(std::unique_ptr<ComponentFactory> _cf);

      /// \brief Add new component to an entity
      /// \param[in] _id The entity that gets the component
      /// \param[out] _api An API instance that will allow read/write access
      /// \returns true if a component was added to the entity
      public: bool AddComponent(EntityId _id, ComponentAPI &_api);

      /// \brief remove a component from an entity
      /// \returns true if the entity had the component and it is now removed
      public: bool RemoveComponent(EntityId _id,
                  gazebo::ecs::ComponentType _type);

      /// \brief Get a component that's on an entity for reading only
      /// \param[in] _id The entity whose component is to be read
      /// \param[out] _api An API instance that will allow read access
      /// \returns true if the entity has the component
      public: bool EntityComponent(EntityId _id, ComponentAPI &_api) const;

      /// \brief Get a component that's on an entity for reading or writing
      /// \param[in] _id The entity that gets the component
      /// \param[out] _api An API instance that will allow read/write access
      /// \returns true if the entity has the component
      public: bool EntityComponentMutable(EntityId _id, ComponentAPI &_api);

      /// \brief Test if a component changed last timestep
      public: Difference IsDifferent(EntityId _id,
                  gazebo::ecs::ComponentType _type) const;

      /// \brief Test hook for instantaneous query results
      public: void InstantQuery(EntityQuery &_query);

      /// \brief Private IMPLementation pointer
      private: std::unique_ptr<EntityComponentDatabasePrivate> dataPtr;
    };
  }
}

#endif
