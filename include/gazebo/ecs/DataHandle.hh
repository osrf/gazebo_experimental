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

#ifndef GAZEBO_ECS_DATAHANDLE_HH_
#define GAZEBO_ECS_DATAHANDLE_HH_

#include <memory>
#include <ignition/common/Time.hh>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/Component.hh"


namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class Manager;

    /// \brief Forward declaration
    class DataHandlePrivate;

    /// \brief A handle to guarantee access to entities and components
    /// \description This handle gives safe access to entities and components.
    ///              As long as this handle exists, it is guaranteed the
    ///              entities and components are immutable.
    class DataHandle
    {
      /// \brief Constructs a handle
      protected: DataHandle(Manager &_manager,
                     EntityComponentDatabase &_database);

      /// \brief destructs a handle
      public: ~DataHandle();

      /// \brief Get the current simulation time
      public: const ignition::common::Time &SimulationTime() const;

      /// \brief Set the simulation time of the next update
      /// \returns true if the time was set, or false if paused
      public: bool SimulationTime(const ignition::common::Time &_newTime);

      /// \brief Creates a new entity
      public: EntityId CreateEntity();

      /// \brief Deletes the given entity
      public: bool DeleteEntity(EntityId _id);

      /// \brief Returns an entity instance with the given ID
      /// \returns Entity with id set to NO_ENTITY if entity does not exist
      public: gazebo::ecs::Entity &Entity(const EntityId _id) const;

      /// \brief private implementation pointer
      private: std::unique_ptr<DataHandlePrivate> dataPtr;

      friend class Manager;
    };
  }
}


#endif
