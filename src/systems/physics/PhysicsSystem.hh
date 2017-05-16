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

#ifndef GAZEBO_SYSTEMS_PHYSICS_PHYSICSSYSTEM_HH_
#define GAZEBO_SYSTEMS_PHYSICS_PHYSICSSYSTEM_HH_

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"


namespace gazebo
{
  namespace systems
  {
    /// \brief ECSystem to do physics
    class PhysicsSystem : public ecs::System
    {
      /// \brief Called when the system is loaded
      /// \param[in] Registrar used to register callbacks for query results
      public: virtual void Init(ecs::QueryRegistrar &_registrar);

      /// \brief Called every physics update with global configuration
      /// \param[in] _result EntityQuery with results fromm a registered query
      protected: void UpdateConfig(const ecs::EntityQuery &_result);

      /// \brief Called every physics update with things to simulate
      /// \param[in] _result EntityQuery with results fromm a registered query
      protected: void UpdateBodies(const ecs::EntityQuery &_result);

      /// \brief max time in seconds to step the world
      protected: double maxStepSize = 0.001;
    };
  }
}
#endif

