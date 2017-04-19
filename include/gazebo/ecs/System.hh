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

#ifndef GAZEBO_ECS_SYSTEM_HH_
#define GAZEBO_ECS_SYSTEM_HH_

#include <memory>

// Could be forward declarations, but systems will include them anyways
#include "gazebo/ecs/QueryRegistrar.hh"
#include "gazebo/ecs/EntityQuery.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief Forward declaration
    class SystemPrivate;

    /// \brief Forward Declaration
    class Manager;

    /// \brief base class for a System
    ///
    /// A System operates on entities that have certain components. A system
    /// will only operate on an entity if it has all of the required components
    class System
    {
      public: System();
      public: ~System();

      /// \brief Initialize the system so it can register queries and callbacks
      public: virtual void Init(QueryRegistrar &_registrar) = 0;

      friend ecs::Manager;

      /// \brief Get the manager this system is a part of
      public: ecs::Manager &Manager();

      /// \brief Set the manager this system is a part of
      private:void Manager(ecs::Manager *_mgr);

      /// \brief No copy constructor
      private: System(const System&) = delete;

      private: std::unique_ptr<SystemPrivate> dataPtr;
    };
  }
}
#endif
