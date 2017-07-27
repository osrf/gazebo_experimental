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

#ifndef GAZEBO_SERVER_COMPONENTFACTORY_HH_
#define GAZEBO_SERVER_COMPONENTFACTORY_HH_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Types.hh>

#include "gazebo/server/Types.hh"

namespace gazebo
{
  namespace server
  {
    /// \brief A virtual case class for storage of a particual component type.
    /// This class, and the ComponentStore derived class, are designed to
    /// store all components of the same type in consecutive memory.
    /// \sa ComponentStore
    class ComponentStoreBase
    {
      /// \brief Default constructor
      public: ComponentStoreBase();

      /// \brief Get the component type stored in this class.
      /// \return Type stored in this component storage class.
      public: ComponentType Type() const;

      /// \brief Create a new component of the type that is stored here.
      /// \param[in] _elem SDF element pointer that contains data for the
      /// component to read an initialize its data.
      /// \return ID of the new component.
      public: virtual ComponentId Create(sdf::ElementPtr _elem) = 0;

      /// \brief Create a new component of the type that is stored here.
      /// \return ID of the new component.
      public: virtual ComponentId Create() = 0;

      /// \brief Remove a component
      /// \param[in] _index Id of the component to remove
      /// \return True on success.
      public: virtual bool Remove(const ComponentId _index) = 0;

      /// \brief The type of components stored in this class
      private: const ComponentType type;

      /// \brief A counter to create unique type Ids
      private: static ComponentType typeCounter;
    };

    /// \brief Implementation of ComponentStoreBase
    /// Any type used in the ComponentStore must have a constructor
    /// that accepts and sdf::ElementPtr.
    /// \sa ComponentStoreBase
    template<typename T>
    class ComponentStore : public ComponentStoreBase
    {
      /// \brief Default Constructor
      /// \params[in] _sdfCreator A function that instantiates a component
      /// using an SDF element pointer.
      public: ComponentStore() = default;

      /// \brief Constructor
      /// \params[in] _sdfCreator A function that instantiates a component
      /// using an SDF element pointer.
      public: ComponentStore(std::function<T (sdf::ElementPtr)> _sdfCreator)
              : sdfCreator(_sdfCreator)
              {
              }

      /// \brief Create a new component of the type that is stored here.
      /// \param[in] _elem SDF element pointer that contains data for the
      /// component to read an initialize its data.
      /// \return ID of the new component.
      public: ComponentId Create(sdf::ElementPtr _elem)
              {
                if (this->sdfCreator)
                {
                  this->components.emplace_back(this->sdfCreator(_elem));
                  return components.size() - 1;
                }
                else
                  return -1;
              }

      /// \brief Create a new component of the type that is stored here.
      /// \return ID of the new component.
      public: ComponentId Create()
              {
                this->components.emplace_back(T());
                return components.size() - 1;
              }

      /// \brief Remove a component
      /// \param[in] _id Index of the component to remove
      /// \return True on success.
      public: bool Remove(const ComponentId _id)
              {
                if (_id < this->components.size())
                {
                  this->components.erase(this->components.begin() + _id);
                  return true;
                }

                return false;
              }

      /// \brief Get a constant pointer to a component
      /// \param[in] _id Id of the component to retrieve.
      /// \return A pointer to the component, or nullptr if the _id is
      /// invalid.
      public: T const *Component(const ComponentId _id)
              {
                if (_id < this->components.size())
                {
                  return &this->components[_id];
                }

                return nullptr;
              }

      /// \brief Get a mutable pointer to a component
      /// \param[in] _id Id of the component to retrieve.
      /// \return A pointer to the component, or nullptr if the _id is
      /// invalid.
      public: T *MutableComponent(const ComponentId _id)
              {
                if (_id < this->components.size())
                {
                  return &this->components[_id];
                }

                return nullptr;
              }

      public: T DefaultComponent() const
              {
                return this->defaultComponent;
              }

      /// \brief Component storage.
      private: std::vector<T> components;

      private: T defaultComponent;

      /// \brief SDF component creator
      private: std::function<T (sdf::ElementPtr)> sdfCreator;
    };

    /// \brief A factort that registers, creates, and manages components.
    class ComponentFactory
    {
      /// \brief Register a Component type with a name
      /// \param[in] _name Name(key) of the type to register.
      /// \return True if the _name has not already been used.
      /// \sa Create
      public: template <typename T>
              static bool Register(const std::vector<std::string> &_names,
                  std::function<T (sdf::ElementPtr)> _creator)
              {
                bool success = false;

                // Check if the component is already registered
                if (!ComponentRegistered<T>())
                {
                  std::lock_guard<std::mutex> lock(mutex);

                  // Create the new component storage container
                  std::unique_ptr<ComponentStoreBase> data(
                      new ComponentStore<T>(_creator));

                  ComponentType type = data->Type();

                  // Store the component storage object
                  componentStores.emplace(type, std::move(data));

                  for (auto const &name : _names)
                  {
                    componentNameTypeMap[name] = type;
                  }
                  success = true;
                }

                return success;
              }

      /// \brief Register a Component type with a name
      /// \param[in] _name Name(key) of the type to register.
      /// \return True if the _name has not already been used.
      /// \sa Create
      public: template <typename T>
              static bool Register(const std::vector<std::string> &_names)
              {
                bool success = false;

                // Check if the component is already registered
                if (!ComponentRegistered<T>())
                {
                  std::lock_guard<std::mutex> lock(mutex);

                  std::unique_ptr<ComponentStoreBase> data(
                      new ComponentStore<T>());

                  ComponentType type = data->Type();

                  // Store the component storage object
                  componentStores.emplace(type, std::move(data));

                  for (auto const &name : _names)
                  {
                    componentNameTypeMap[name] = type;
                  }
                  success = true;
                }

                return success;
              }

      /// \brief Get whether the typename is already registered as
      /// a component.
      /// \return True if typename is registered.
      public: template <typename T>
              static bool ComponentRegistered()
              {
                std::lock_guard<std::mutex> lock(mutex);
                bool result = false;
                for (auto const &cs : componentStores)
                {
                  ComponentStore<T> *store = nullptr;
                  try
                  {
                    store = dynamic_cast<ComponentStore<T>*>(cs.second.get());
                  }
                  catch (...)
                  {
                    continue;
                  }

                  if (store)
                  {
                    result = true;
                    break;
                  }
                }

                return result;
              }


      /// \brief Get a ComponentType that matches a component type name
      /// \param[in] _name Name of a component, such as "pose".
      /// \return A ComponenType, or NO_COMPONENT if the _name is not known.
      public: static ComponentType Type(const std::string &_name);

      /// \brief Return a ComponentType for a component by actual type
      /// \return The ComponentType id that matches the provided template
      /// type, or NO_COMPONENT if the template type is not known.
      public: template <typename T>
              static ComponentType Type()
              {
                ComponentStore<T> *store = RegisteredComponent<T>();
                std::lock_guard<std::mutex> lock(mutex);

                if (store)
                  return store->Type();
                else
                  return NO_COMPONENT;
              }

      /// \brief Create a new component, based on information in an SDF
      /// Element
      /// \param[in] _elem The SDF element to create a component from.
      /// \return True on success
      public: static bool CreateComponent(EntityId _id, sdf::ElementPtr _elem);

      public: template<typename T>
              static T *CreateComponent(EntityId _id)
              {
                ComponentStore<T> *store = RegisteredComponent<T>();
                if (!store)
                  return nullptr;

                // Create the component
                auto id = store->Create();

                // Match the component to the entity, adding the entity if
                // it doesn't exist
                entityComponentMap[_id].push_back(
                    std::make_pair(store->Type(), id));

                // Return a pointer to the new component
                return store->MutableComponent(id);
              }

      /// \brief Remove all components that belong to an Entity.
      /// \param[in] _id Id of the Entity.
      /// \return True if the components were successfully removed.
      public: static bool RemoveComponents(EntityId _id);

      /// \brief Remove all components of a type T from an Entity.
      /// \param[in] _id Id of the Entity.
      /// \return True if the components were successfully removed.
      public: template<typename T>
              static bool RemoveComponent(EntityId _id)
              {
                if (entityComponentMap.find(_id) == entityComponentMap.end())
                  return false;

                bool success = true;
                ComponentType type = Type<T>();

                for (auto &ec : entityComponentMap[_id])
                {
                  if (ec.first == type)
                    success = success && RemoveComponent(ec);
                }

                return success;
              }

      /// \brief Get a component type that belongs to an entity.
      /// \todo: There could be multiple components of the same type, and
      /// this function should return a list.
      /// \param[in] _id Id of the Entity.
      /// \return Constant pointer to the component, or nullptr if the
      /// component or entity do not exist.
      public: template<typename T>
              static T const *Component(EntityId _id)
              {
                // check for the entity
                if (entityComponentMap.find(_id) != entityComponentMap.end())
                {
                  ComponentType type = Type<T>();

                  // Find the component
                  for (auto &ec : entityComponentMap[_id])
                  {
                    if (ec.first == type)
                    {
                      return static_cast<ComponentStore<T>*>(
                          componentStores[type].get())->Component(ec.second);
                    }
                  }

                  return nullptr;
                }

                return nullptr;
              }

      /// \brief Get a component type that belongs to an entity.
      /// \todo: There could be multiple components of the same type, and
      /// this function should return a list.
      /// \param[in] _id Id of the Entity.
      /// \return Mutable pointer to the component, or nullptr if the
      /// component or entity do not exist.
      public: template<typename T>
              T *MutableComponent(EntityId _id)
              {
                if (entityComponentMap.find(_id) != entityComponentMap.end())
                {
                  ComponentType type = Type<T>();

                  /// \todo: Check that _id exists
                  for (auto &ec : entityComponentMap[_id])
                  {
                    if (ec.first == type)
                    {
                      return static_cast<ComponentStore<T>*>(
                          componentStores[type].get())->MutableComponent(
                          ec.second);
                    }
                  }

                  return nullptr;
                }

                return nullptr;
              }

      /// \brief Checks if an Entity contains the set of components.
      /// \param[in] _id Id of the entity.
      /// \param[in] _types A set of types to check.
      /// \return True if the entity exists and has the specified set of
      /// components.
      public: static bool EntityMatches(EntityId _id,
                  const std::set<ComponentType> &_types);

      /// \brief Remove a component.
      /// \param[in] _key A key into the entityComponentMap vector.
      /// \return True on success.
      private: static bool RemoveComponent(
                  const std::pair<ComponentType, ComponentId> &_key);

      /// \brief Get the component store that is associated with the give
      /// typename
      /// \return Pointer to the component store, or nullptr on error.
      private: template <typename T>
              static ComponentStore<T> *RegisteredComponent()
              {
                std::lock_guard<std::mutex> lock(mutex);
                bool result = false;
                for (auto const &cs : componentStores)
                {
                  ComponentStore<T> *store = nullptr;
                  try
                  {
                    store = dynamic_cast<ComponentStore<T>*>(cs.second.get());
                  }
                  catch (...)
                  {
                    continue;
                  }

                  if (store)
                  {
                    return store;
                  }
                }

                return nullptr;
              }

      /// \brief Lock for thread safety
      private: static std::mutex mutex;

      /// \brief The component storage objects.
      private: static std::map<ComponentType,
               std::unique_ptr<ComponentStoreBase>> componentStores;

      /// A map of EntityID to component identifier.
      /// Key: EntityID
      /// Data: vector of component information
      private: static std::map<EntityId,
               std::vector<
                 std::pair<ComponentType, ComponentId>>> entityComponentMap;

      /// \brief A mape of component names to component types.
      private: static std::map<std::string, ComponentType> componentNameTypeMap;
    };
  }
}

#endif
