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

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

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

      /// \brief Initialize physics world
      /// \returns true if initialization succeeded
      private: void InitBullet();

      /// \brief Creates a Sphere in the bullet world
      private: void CreateRigidBody(ecs::Entity _entity);

      private: std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig;
      private: std::unique_ptr<btCollisionDispatcher> dispatcher;
      private: std::unique_ptr<btBroadphaseInterface> broadPhase;
      private: std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
      private: std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;
      private: std::unordered_map<
                ecs::EntityId,
                std::unique_ptr<btCollisionShape> > collisionShapes;
      private: std::unordered_map<
                ecs::EntityId,
                std::unique_ptr<btRigidBody> > rigidBodies;
      private: std::unordered_map<
                ecs::EntityId,
                std::unique_ptr<btMotionState> > motionStates;
    };
  }
}
#endif

