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

#ifndef GAZEBO_ECS_ENTITY_HH_
#define GAZEBO_ECS_ENTITY_HH_

#include <memory>
#include "gazebo/ecs/ComponentFactory.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief An entity is an id!
    typedef int EntityId;

    /// \brief Describes the changes to an entity or component
    enum Difference {
      NO_DIFFERENCE = 0,
      WAS_CREATED = 1,
      WAS_DELETED = 2,
      WAS_MODIFIED = 3
    };

    /// \brief For results which there is no entity
    const EntityId NO_ENTITY = -1;

    /// \brief Forward Declaration
    class EntityPrivate;

    /// \brief Forward Declaration
    class EntityComponentDatabase;

    /// \brief A convenience class for working with entities
    class Entity
    {
      public: Entity();

      public: Entity(Entity &&_entity);

      /// \brief Move assignment operator
      public: Entity &operator=(Entity &&_entity);

      /// \brief Destructor
      public: ~Entity();

      /// \brief Return id of entity
      public: EntityId Id() const;

      /// \brief Get a component by actual type for reading
      /// \returns pointer to a component or nullptr on error
      public: template <typename T>
              T const *Component()
              {
                auto type = ComponentFactory::Type<T>();
                return static_cast<T const *>(this->Component(type));
              }

      /// \brief Get a component by actual type for reading or writing
      /// \returns pointer to a component or nullptr on error
      public: template <typename T>
              T *ComponentMutable()
              {
                auto type = ComponentFactory::Type<T>();
                return static_cast<T*>(this->ComponentMutable(type));
              }

      /// \brief Get a component by ComponentType for reading
      /// \returns pointer to a component or nullptr on error
      private: void const *Component(const ComponentType&);

      /// \brief Get a component by ComponentType for reading or writing
      /// \returns pointer to a component or nullptr on error
      private: void *ComponentMutable(const ComponentType&);

      /// \brief Add a component by actual type
      /// \returns pointer to a component or nullptr on error
      public: template <typename T>
              T *AddComponent()
              {
                auto type = ComponentFactory::Type<T>();
                return static_cast<T*>(this->AddComponent(type));
              }

      /// \brief Add a component by ComponentType
      /// \returns pointer to a component or nullptr on error
      public: void *AddComponent(const ComponentType&);

      /// \brief Test if a component is different by actual type
      /// \returns the difference if any
      public: template <typename T>
              Difference IsDifferent() const
              {
                auto type = ComponentFactory::Type<T>();
                return this->IsDifferent(type);
              }

      /// \brief Test if a component changed last timestep
      /// \param[in] _type Type of component to check
      public: Difference IsDifferent(ComponentType _type) const;

      /// \brief Private data pointer
      private: std::unique_ptr<EntityPrivate> dataPtr;

      /// \brief Constructor. Use Manager::CreateEntity() instead
      private: Entity(EntityComponentDatabase *_mgr, EntityId _id);

      /// \brief friendship
      friend EntityComponentDatabase;
    };

    static Entity EntityNull;
  }
}
#endif
