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

#ifndef GAZEBO_ECS_ENTITYQUERY_HH_
#define GAZEBO_ECS_ENTITYQUERY_HH_

#include <memory>
#include <vector>
#include <set>
#include <type_traits>
#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/Component.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class EntityQueryPrivate;

    /// \brief Forward declaration
    class EntityComponentDatabase;

    /// \brief a Class for querying entities from a manager
    class EntityQuery
    {
      /// \brief Constructor
      public: EntityQuery();

      /// \brief Destructor
      public: ~EntityQuery();

      /// \brief Return true if this is an empty/null entity query.
      public: bool IsNull();

      /// \brief Add a component to the query
      /// \return True if the type was successfully added.
      public: template <typename T>
        bool AddComponent()
        {
          static_assert(std::is_base_of<ComponentAPI, T>::value,
              "AddComponent expects subclass of gazebo::ecs::ComponentAPI");
          T comp;
          return this->AddComponent(comp.ComponentType());
        }

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

      /// \brief Clear results of a query. This will keep the set of
      /// component, and clear the set of entities.
      private: void Clear();

      /// \brief friendship
      private: friend EntityComponentDatabase;

      /// \brief Private data pointer
      private: std::shared_ptr<EntityQueryPrivate> dataPtr;
    };

    static const EntityQuery EntityQueryNull;
  }
}
#endif
