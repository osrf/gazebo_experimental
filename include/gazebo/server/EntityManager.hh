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
#include <sdf/sdf.hh>

#include "gazebo/server/ComponentFactory.hh"
#include "gazebo/server/Types.hh"
#include "gazebo/server/Entity.hh"

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

      /// \brief Create a new component, based on information in an SDF Element
      /// \param[in] _elem The SDF element to create a component from.
      /// \return True on success
      public: bool CreateComponent(EntityId _id, sdf::ElementPtr _elem);

      /// \brief return true iff the entity exists
      public: bool EntityExists(EntityId _id) const;

      /// \brief Get a component type that belongs to an entity.
      /// \todo: There could be multiple components of the same type, and
      /// this function should return a list.
      /// \param[in] _id Id of the Entity.
      /// \return Constant pointer to the component, or nullptr if the
      /// component or entity do not exist.
      public: template<typename T>
              T const *Component(EntityId _id)
              {
                // check for the entity
                if (this->EntityExists(_id))
                {
                  ComponentType type = ComponentFactory::Type<T>();

                  // Find the component
                  for (auto const &ec : this->Components(_id))
                  {
                    if (ec.first == type)
                    {
                      return ComponentFactory::Component<T>(ec.second);
                    }
                  }

                  return nullptr;
                }

                return nullptr;
              }

      /// \brief Get a mutable component type that belongs to an entity.
      /// \todo: There could be multiple components of the same type, and
      /// this function should return a list.
      /// \param[in] _id Id of the Entity.
      /// \return Constant pointer to the component, or nullptr if the
      /// component or entity do not exist.
      public: template<typename T>
              T const *MutableComponent(EntityId _id)
              {
                // check for the entity
                if (this->EntityExists(_id))
                {
                  ComponentType type = ComponentFactory::Type<T>();

                  // Find the component
                  for (auto const &ec : this->Components(_id))
                  {
                    if (ec.first == type)
                    {
                      return ComponentFactory::MutableComponent<T>(ec.second);
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
      public: bool EntityMatches(const EntityId _id,
                  const std::set<ComponentType> &_types);

      /// \brief Remove all components that belong to an Entity.
      /// \param[in] _id Id of the Entity.
      /// \return True if the components were successfully removed.
      public: bool RemoveComponents(const EntityId _id);

      /// \brief Remove all components of a type from an entity.
      /// \param[in] _id Id of the entity.
      /// \param[in] _type Component type.
      /// \return True on success.
      public: bool RemoveComponent(const EntityId _id,
                  const ComponentType _type);

      /// \brief Remove a component from an entity.
      /// \param[in] _id Id of the entity.
      /// \param[in] _type Component type.
      /// \param[in] _compId Component id.
      /// \return True on success.
      public: bool RemoveComponent(const EntityId _id,
                  const ComponentType _type, const ComponentId _compId);

      /// \brief Get a component that is attached to an entity.
      /// \param[in] _id Id of the entity.
      /// \param[in] _compType Type of component
      /// \return Id of the component
      public: ComponentId Component(const EntityId _id,
                   const ComponentType _compType) const;

      /// \brief Attach a component to an entity.
      /// \param[in] _id Id of the entity.
      /// \param[in] _compType Type of component
      /// \param[in] _compId Id of the component
      public: void AddComponent(const EntityId _id,
                   const ComponentType _compType, const ComponentId _compId);

      /// \brief Get the vector of component information associated with an
      /// entity.
      /// Note: This internal function assumes that the caller has checked for
      /// the existence of _id
      /// \return Component information for the entity.
      private: const std::vector<std::pair<ComponentType, ComponentId>> &
               Components(const EntityId _id) const;

      /// \brief Private Implementation pointer
      private: std::unique_ptr<EntityManagerPrivate> dataPtr;
    };
  }
}

#endif
