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

#include "gazebo/components/WorldPose.hh"
#include "gazebo/components/WorldVelocity.hh"
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
      /// \brief Called when the system is loaded
      public: virtual void Init(ecs::QueryRegistrar &_registrar);

      /// \brief Called every physics update
      /// \param[in] _dt Time step
      /// \param[in] _result An EntityQuery with result from the query
      /// returned from Init().
      /// \param[in] _mgr An ecs::Manager instance that can be used to
      /// add/delete entities.
      public: virtual void Update(const ecs::EntityQuery &_result);

      /// \brief Physics systems need to bridge between entities/components
      ///        and their internal reprentations of the world
      private: dumb_physics::World world;

      /// \brief Sets internal representation to match component
      /// \param[in] _body Internal representation of a body
      /// \param[in] _component ECS sphere geometry component
      private: void SyncInternalGeom(dumb_physics::Body *_body,
                   const components::Geometry *_component);

      /// \brief Sets internal representation to match component
      /// \param[in] _body Internal representation of a body
      /// \param[in] _component ECS inertial component. Only the mass is used
      /// by this system.
      private: void SyncInternalMass(dumb_physics::Body *_body,
                   const components::Inertial *_component);

      /// \brief Sets internal representation to match component
      /// \param[in] _body Internal representation of a body
      /// \param[in] _component ECS world velocity component. Both lienar and
      /// angular are handled.
      private: void SyncInternalVelocity(dumb_physics::Body *_body,
                   const components::WorldVelocity *_component);

      /// \brief Sets internal representation to match component
      /// \param[in] _body Internal representation of a body
      /// \param[in] _component ECS world pose component
      private: void SyncInternalPose(dumb_physics::Body *_body,
                   const components::WorldPose *_component);

      /// \brief Sets component to match internal representation
      /// \param[in] _body External representation of a body
      /// \param[in] _component ECS sphere geometry component
      private: void SyncExternalGeom(const dumb_physics::Body *_body,
                   components::Geometry *_component);

      /// \brief Sets component to match internal representation
      /// \param[in] _body External representation of a body
      /// \param[in] _component ECS inertial component. Only the mass is used
      /// by this system.
      private: void SyncExternalMass(const dumb_physics::Body *_body,
                   components::Inertial *_component);

      /// \brief Sets component to match internal representation
      /// \param[in] _body External representation of a body
      /// \param[in] _component ECS world velocity component. Both lienar and
      /// angular are handled.
      private: void SyncExternalVelocity(const dumb_physics::Body *_body,
                   components::WorldVelocity *_component);

      /// \brief Sets component to match internal representation
      /// \param[in] _body External representation of a body
      /// \param[in] _component ECS world pose component
      private: void SyncExternalPose(const dumb_physics::Body *_body,
                   components::WorldPose *_component);

      /// \brief Adds a body to the world
      private: dumb_physics::Body *AddBody(const ecs::EntityId _id,
                                           ecs::Entity &_entity);
    };
  }
}
#endif

