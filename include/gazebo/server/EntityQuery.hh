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
#ifndef GAZEBO_SERVER_ENTITYQUERY_HH_
#define GAZEBO_SERVER_ENTITYQUERY_HH_

#include <memory>
#include <vector>
#include <set>
#include "gazebo/server/Entity.hh"
#include "gazebo/server/ComponentFactory.hh"

namespace gazebo
{
  namespace server
  {
    /// \brief Forward declaration
    class EntityQueryPrivate;

    /// \brief a Class for querying entities from a manager
    class EntityQuery
    {
      /// \brief Constructor
      public: EntityQuery();

      /// \brief Move Constructor
      public: EntityQuery(EntityQuery &&_query);

      /// \brief Destructor
      public: ~EntityQuery();

      /// \brief Return true if this is an empty/null entity query.
      public: bool IsNull();

      /// \brief Add a component based on a name.
      /// \param[in] _name Name of the component to add. This will look up
      /// the component in the ComponentFactory.
      /// \return True on success, which means the component was found in
      /// the ComponentFactory.
      /// \todo templated version
      public: bool AddComponent(const std::string &_name);

      /// \brief Add a component based on a component type.
      /// \param[in] _type Type of component to add.
      /// \return True if the _type was successfully added.
      public: bool AddComponent(ComponentType _type);

      /// \brief Get the components that have been added to the query.
      /// \return A const reference to the set of components in this query.
      public: const std::set<ComponentType> &ComponentTypes() const;

      /// \brief Returns true if these are the same queries.
      /// \param[in] _rhs The right hand side argument.
      /// \return True if this query matches _rhs.
      public: bool operator==(const EntityQuery &_rhs) const;

      public: EntityQuery &operator=(EntityQuery &&_entityQuery);

      public: bool HasEntity(const EntityId _id) const;

      /// \brief Add an entity.
      /// \param[in] _id Id of the entity to add.
      /// \return True if the entity was added.
      public: bool AddEntity(const EntityId _id);

      /// \brief Remove an entity.
      /// \param[in] _id Id of the entity
      /// \return True if the entity was removed.
      public: void RemoveEntity(const EntityId _id);

      /// \brief Get the entity ids that match this query.
      /// \todo ordered results matching component placement in memory
      /// \return The entities that match the components in this query.
      public: const std::set<EntityId> &EntityIds() const;

      /// \brief Get an entity from an entity id
      /// \param[in] _id Id of the entity to retrieve
      /// \return A reference to the entity
      public: Entity &EntityById(const EntityId _id) const;

      /// \brief Internal use only
      public: void SetEntityCb(std::function<Entity &(const EntityId)> _func);

      /// \brief Clear results of a query. This will keep the set of
      /// component, and clear the set of entities.
      private: void Clear();

      /// \brief Private data pointer
      private: std::unique_ptr<EntityQueryPrivate> dataPtr;
    };

    static const EntityQuery NullEntityQuery;
  }
}
#endif
