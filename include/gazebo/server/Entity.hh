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

#ifndef GAZEBO_SERVER_ENTITY_HH_
#define GAZEBO_SERVER_ENTITY_HH_

#include <memory>
#include <ignition/common/Types.hh>
#include "gazebo/server/ComponentManager.hh"
#include "gazebo/server/Types.hh"

namespace gazebo
{
  namespace server
  {
    class EntityManager;

    /// \brief Forward Declaration
    class EntityPrivate;

      /// \brief A convenience class for working with entities
    class Entity
    {
      /// \brief Constructor
      public: Entity();

      /// \brief Construct an Entity with a particular id
      /// \param[in] _id Id of this new Entity.
      public: Entity(const EntityId _id);

      /// \brief Deleted copy constructor
      public: Entity(const Entity &_entity) = delete;

      /// \brief Move constructor
      /// \param[in] _entity Entity to move.
      public: Entity(Entity &&_entity);

      /// \brief Destructor
      public: ~Entity();

      /// \brief Move assignment operator
      /// \param[in] _entity Entity to move.
      /// \return Reference to this Entity.
      public: Entity &operator=(Entity &&_entity);

      /// \brief Equality operator
      /// \return True if the id's are the same
      public: bool operator==(const Entity &_entity) const;

      /// \brief Return id of entity
      /// \return ID of this entity
      public: EntityId Id() const;

      /// \brief Get a component by typename
      /// \returns Constant pointer to a component or nullptr on error
      public: template <typename T>
              const T *Component()
              {
                // Get the component type
                ComponentType type = ComponentManager::Type<T>();

                // Get the component id
                ComponentId compId = this->ComponentIdMatchingType(type);

                return ComponentManager::Component<T>(compId);
              }


      /// \brief Get a component by typename
      /// \returns Mutable pointer to a component or nullptr on error
      public: template <typename T>
              T *MutableComponent()
              {
                // Get the component type
                ComponentType type = ComponentManager::Type<T>();

                // Get the component id
                ComponentId compId = this->ComponentIdMatchingType(type);

                return ComponentManager::MutableComponent<T>(compId);
              }

      /// \brief Add a component by type
      /// \return Pointer to the new component or nullptr on error
      public: template <typename T>
              T *AddComponent()
              {
                // Get the component type
                ComponentType type = ComponentManager::Type<T>();

                // Create the component
                ComponentId compId = ComponentManager::CreateComponent(type);

                // Connect this entity to the component
                this->AddComponent(type, compId);

                // Return the actual component data.
                return ComponentManager::MutableComponent<T>(compId);
              }

      /// \brief Remove a component from this entity by type
      /// \return True on success
      public: template <typename T>
              bool RemoveComponent()
              {
                // Get the component type
                ComponentType type = ComponentManager::Type<T>();

                // Get the component id
                ComponentId compId = this->ComponentIdMatchingType(type);

                return this->RemoveComponent(type, compId);
              }

      /// \todo: Use a weakpointer here.
      public: void SetManager(EntityManager *_mgr);

      private: ComponentId ComponentIdMatchingType(const ComponentType _type);

      private: void AddComponent(const ComponentType _type,
                   const ComponentId _compId);

      private: bool RemoveComponent(const ComponentType _type,
                   const ComponentId _compId);

      /// \brief Private data pointer
      private: std::unique_ptr<EntityPrivate> dataPtr;
    };

    /// \brief A Null Entity.
    static Entity NullEntity;
  }
}
#endif
