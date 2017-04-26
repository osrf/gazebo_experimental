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

#ifndef GAZEBO_ECS_MANAGER_HH_
#define GAZEBO_ECS_MANAGER_HH_

#include <memory>
#include <iostream>
#include <set>

#include <ignition/common/Time.hh>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"
#include "gazebo/ecs/ComponentFactory.hh"


namespace gazebo
{
  namespace ecs
  {
    // Forward declare private data class.
    class ManagerPrivate;

    class Manager
    {
      public: Manager();
      public: ~Manager();

      /// \brief Get the current simulation time
      public: const ignition::common::Time &SimulationTime();

      /// \brief Set the simulation time of the next update
      /// \returns true if the time was set, or false if paused
      public: bool SimulationTime(const ignition::common::Time &_newTime);

      /// \brief Pause the simulation
      /// \description EndPause() must be called for each call to BeginPause().
      /// \returns a count of systems that want time paused. This will never be
      ///   less than 1.
      public: int BeginPause();

      /// \brief Unpause the simulation
      /// \returns a count of systems that want time paused.
      public: int EndPause();

      /// \brief Check if simulation is paused
      /// \returns true if the simulation is paused
      public: bool Paused();

      /// \brief Creates a new entity
      public: EntityId CreateEntity();

      /// \brief Deletes the given entity
      public: bool DeleteEntity(EntityId _id);

      /// \brief Convenience function to load a system from a type
      ///
      /// Ex: sm->LoadSystem<FancySystemClass>();
      public: template <typename T>
              bool LoadSystem()
              {
                return this->LoadSystem(std::unique_ptr<System>(new T()));
              }

      /// \brief Load a system
      ///
      /// Ex: sm->LoadSystem(std::move(aUniquePtrInstance))
      public: bool LoadSystem(std::unique_ptr<System> _sys);

      public: void UpdateSystems();

      /// \brief Returns an entity instance with the given ID
      /// \returns Entity with id set to NO_ENTITY if entity does not exist
      public: gazebo::ecs::Entity &Entity(const EntityId _id) const;

      private: Manager(const Manager&) = delete;

      private: std::unique_ptr<ManagerPrivate> dataPtr;

      private: friend class Entity;
    };
  }
}
#endif
