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

#ifndef GAZEBO_SERVER_MANAGER_HH_
#define GAZEBO_SERVER_MANAGER_HH_

#include <memory>
#include <iostream>
#include <set>

#include <ignition/common/Time.hh>

#include "gazebo/server/Entity.hh"
#include "gazebo/server/System.hh"

namespace gazebo
{
  namespace server
  {
    /// \brief Forward declare private data class.
    class ManagerPrivate;

    class Manager
    {
      /// \brief Constructor
      public: Manager();

      /// \brief Destructor
      public: ~Manager();

      /// \brief Load the Manager.
      public: bool Load(const std::string &_worldFile);

      /// \brief Load a system
      ///
      /// Ex: sm->LoadSystem("my_system")
      public: bool LoadSystem(const std::string &_name);

      /// \brief Load a system
      ///
      /// Ex: sm->LoadSystem({"my_system", "your_system"})
      public: bool LoadSystems(const std::vector<std::string> &_libs);

      /// \brief Run the manager, and all systems
      public: void Run();

      /// \brief Stop the manager, and all systems
      public: void Stop();

      /// \brief Get the current simulation time
      /// \return The current simulation time
      public: const ignition::common::Time &SimulationTime() const;

      /// \brief Set the simulation time of the next update
      /// \returns true if the time was set, or false if paused
      public: bool SetSimulationTime(const ignition::common::Time &_newTime);

      /// \brief Set whether simulation is paused
      public: void SetPaused(const bool _pause);

      /// \brief Check if simulation is paused
      /// \returns true if the simulation is paused
      public: bool Paused() const;

      /// \brief Update everything once and return immediately
      public: void UpdateOnce();

      /// \brief Update everything once, then sleep to achieve a desired
      ///        real time factor.
      /// \remark Even if this method is called in a tight loop simulation,time
      //          will very slightly lag behind wall clock time. This is due to
      //          the sleep time calculations not considering the time it takes
      //          to calculate the amount of time to sleep before sleeping.
      /// \param[in] _real_time_factor ratio of sim time to wall clock time
      public: void UpdateOnce(const double _realTimeFactor);

      /// \brief Create an entity.
      /// \return The new entity Id.
      public: Entity &CreateEntity(const std::string &_name);

      /// \brief Get an Entity instance by Id
      /// \param[in] _id Id of the Entity to retrieve.
      /// \return Reference to the Entity
      public: ::gazebo::server::Entity &EntityById(const EntityId _id) const;

      public: ignition::math::graph::UndirectedGraph<EntityId, int> &
              Graph() const;

      /// \brief Implementation of the Run function.
      private: void RunImpl();

      /// Delete the copy constructor
      private: Manager(const Manager &) = delete;

      private: std::unique_ptr<ManagerPrivate> dataPtr;
    };
  }
}
#endif
