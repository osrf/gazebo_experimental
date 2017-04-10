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

#ifndef GAZEBO_PRIVATE_SYSTEMS_DUMBPHYSICS_HH_
#define GAZEBO_PRIVATE_SYSTEMS_DUMBPHYSICS_HH_

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"
#include "dumb_physics/World.hh"
#include "dumb_physics/Body.hh"


namespace gazebo
{
  namespace systems
  {

    /// \brief ECSystem to do simple physics
    ///
    /// The goal of this class is to feel out what kind of features
    /// a physics system will need in an ECS architecture
    class DumbPhysics : public ecs::System
    {
      /// \brief called when the system is loaded
      public: virtual ecs::EntityQuery Init();

      /// \brief called every physics update
      public: virtual void Update(const double _dt,
                  const ecs::EntityQuery &_result,
                  ecs::Manager &_mgr);

      /// \brief Physics systems need to bridge between entities/components
      ///        and their internal reprentations of the world
      private: dumb_physics::World world;

      /// \brief sets internal representation to match component
      private: void SyncBodies(dumb_physics::Body *body,
                   const components::RigidBody *component);

      /// \brief sets internal representation to match component
      private: void SyncVelocity(dumb_physics::Body *body,
                   const components::WorldVelocity *component);

      /// \brief Adds a body to the world
      private: dumb_physics::Body *AddBody(ecs::EntityId _id,
                   const components::RigidBody *bodyComponent,
                   const components::WorldPose *poseComponent);
    };
  }
}
#endif

