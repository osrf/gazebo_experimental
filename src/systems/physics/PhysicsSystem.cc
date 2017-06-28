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

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <gazebo/components/Inertial.api.hh>
#include <gazebo/components/Geometry.api.hh>
#include <gazebo/components/Collidable.api.hh>
#include <gazebo/components/PhysicsConfig.api.hh>
#include <gazebo/components/Pose.api.hh>
#include <gazebo/components/WorldVelocity.api.hh>
#include <gazebo/ecs/Manager.hh>
#include <gazebo/ecs/EntityQuery.hh>
#include "PhysicsSystem.hh"

namespace gzsys = gazebo::systems;
using namespace gzsys;
using namespace gazebo;


//////////////////////////////////////////////////
/// \brief Class updates pose and velocity info for bodies that move
class ComponentMotionState : public btMotionState
{
  /// \brief pointer to a manager
  public: ecs::Manager *mgr;

  /// \brief entity id with pose this motion state should track
  public: ecs::EntityId id;

  /// \brief constructor
  public: ComponentMotionState(ecs::EntityId _id, ecs::Manager *_mgr)
          : id(_id), mgr(_mgr)
    {
    }

    virtual ~ComponentMotionState()
    {
    }

    virtual void getWorldTransform(btTransform &worldTrans) const
    {
      auto &e = this->mgr->Entity(this->id);
      auto pose = e.Component<components::Pose>();
      assert(pose);

      auto &pos = pose.Origin().Pos();
      auto &rot = pose.Origin().Rot();

      worldTrans.setOrigin(btVector3(pos.X(), pos.Y(), pos.Z()));
      worldTrans.setRotation(btQuaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));

      // TODO update velocity component here too
    }

    virtual void setWorldTransform(const btTransform &worldTrans)
    {
      auto &e = this->mgr->Entity(this->id);
      auto pose = e.ComponentMutable<components::Pose>();
      assert(pose);

      btQuaternion rot = worldTrans.getRotation();
      btVector3 pos = worldTrans.getOrigin();
      auto &pos_ign = pose.Origin().Pos();
      auto &rot_ign = pose.Origin().Rot();

      pos_ign.X() = pos.x();
      pos_ign.Y() = pos.y();
      pos_ign.Z() = pos.z();
      rot_ign.W() = rot.w();
      rot_ign.X() = rot.x();
      rot_ign.Y() = rot.y();
      rot_ign.Z() = rot.z();

      // TODO update velocity component here too
    }
};


/////////////////////////////////////////////////
void PhysicsSystem::Init(ecs::QueryRegistrar &_registrar)
{
  // Query for global/configuration info
  ecs::EntityQuery configQuery;
  if (!configQuery.AddComponent<gazebo::components::PhysicsConfig>())
  {
    std::cerr << "Undefined component[gazebo::components::PhysicsConfig]\n";
  }
  else
  {
    _registrar.Register(configQuery,
        std::bind(&PhysicsSystem::UpdateConfig, this, std::placeholders::_1));
  }

  // Query for bodies to simulate
  ecs::EntityQuery query;
  if (!query.AddComponent<gazebo::components::Geometry>())
    ignerr << "Undefined component[gazebo::components::Geometry]\n";
  if (!query.AddComponent<gazebo::components::Pose>())
    ignerr << "Undefined component[gazebo::components::Pose]\n";
  if (!query.AddComponent<gazebo::components::Collidable>())
    ignerr << "Undefined component[gazebo::components::Collidable]\n";

  // Note that we add only the required components. This system will also make
  // use of the Inertial and WorldVelocity components if present, but these are
  // optional

  this->InitBullet();

  _registrar.Register(query,
      std::bind(&PhysicsSystem::UpdateBodies, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void PhysicsSystem::InitBullet()
{
  this->collisionConfig.reset(new btDefaultCollisionConfiguration());
  this->dispatcher.reset(new btCollisionDispatcher(
        this->collisionConfig.get()));
  this->broadPhase.reset(new btDbvtBroadphase());
  this->solver.reset(new btSequentialImpulseConstraintSolver);
  this->dynamicsWorld.reset(new btDiscreteDynamicsWorld(
        this->dispatcher.get(), this->broadPhase.get(), this->solver.get(),
        this->collisionConfig.get()));
}

/////////////////////////////////////////////////
void PhysicsSystem::UpdateConfig(const ecs::EntityQuery &_result)
{
  ecs::Manager &mgr = this->Manager();
  auto const &entityIds = _result.EntityIds();
  if (!entityIds.empty())
  {
    // only consider the first entity
    auto &entity = mgr.Entity(*entityIds.begin());
    auto difference = entity.IsDifferent<components::PhysicsConfig>();
    if (difference == ecs::WAS_CREATED || difference == ecs::WAS_MODIFIED)
    {
      auto config = entity.Component<components::PhysicsConfig>();
      this->maxStepSize = config.MaxStepSize();
    }
  }
}

//////////////////////////////////////////////////
void PhysicsSystem::CreateRigidBody(ecs::Entity &_entity)
{
  auto collidable = _entity.Component<components::Collidable>();
  auto geom = _entity.Component<components::Geometry>();
  auto inertial = _entity.Component<components::Inertial>();
  auto velocity = _entity.Component<components::WorldVelocity>();
  auto pose = _entity.Component<components::Pose>();

  // Todo support multiple collisions on the same link
  if (collidable.GroupId() != ecs::NO_ENTITY)
  {
    ignwarn << "Compound geometries not yet supported " << collidable.GroupId() << std::endl;
    return;
  }

  // Todo support more than just spheres
  if (!geom.Shape().HasSphere())
  {
    ignwarn << "Geometry not supported" << std::endl;
    return;
  }

  const double radius = geom.Shape().Sphere().Radius();

  std::unique_ptr<btCollisionShape> colShape(
      new btSphereShape(radius));

  /// Create Dynamic Objects
  btTransform startTransform;
  startTransform.setIdentity();

  double mass = 0;
  btVector3 localInertia(0, 0, 0);

  if (velocity && inertial)
  {
    mass = inertial.Mass();
    // Dynamic
    // TODO Inertia
    //  http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=3667
    colShape->calculateLocalInertia(mass, localInertia);
  }
  // TODO else velocity means kinematic

  std::unique_ptr<btMotionState> myMotionState(
      new ComponentMotionState(_entity.Id(), &(this->Manager())));

  btRigidBody::btRigidBodyConstructionInfo rbInfo(
    mass, myMotionState.get(), colShape.get(), localInertia);
  std::unique_ptr<btRigidBody> body(new btRigidBody(rbInfo));

  this->dynamicsWorld->addRigidBody(body.get());

  auto id = _entity.Id();

  // Stuffing the entity id into user data pointer, make sure it fits
  static_assert(sizeof(void*) >= sizeof(decltype(id)),
      "Bullet userdata is too small for an entity id");
  body->setUserPointer(reinterpret_cast<void *>(id));

  this->collisionShapes[id] = std::move(colShape);
  this->rigidBodies[id] = std::move(body);
  this->motionStates[id] = std::move(myMotionState);
}

/////////////////////////////////////////////////
void PhysicsSystem::UpdateBodies(const ecs::EntityQuery &_result)
{
  ecs::Manager &mgr = this->Manager();

  // STEP 1 Loop through entities and update internal representation
  // This is where the effects of other systems get propagated to this one,
  // for example, if a pose is changed or a body is deleted through the GUI.
  for (auto const &entityId : _result.EntityIds())
  {
    // Get entity (should check if exists?)
    auto &entity = mgr.Entity(entityId);

    // Check for changes since last time step
    auto diffGeometry = entity.IsDifferent<components::Geometry>();
    auto diffPose = entity.IsDifferent<components::Geometry>();
    auto diffCollidable = entity.IsDifferent<components::Collidable>();

    auto doDelete = (diffGeometry == ecs::WAS_DELETED)
      || (diffPose == ecs::WAS_DELETED)
      || (diffCollidable == ecs::WAS_DELETED);
    auto doCreate = (!doDelete) && (diffGeometry == ecs::WAS_CREATED
        || diffPose == ecs::WAS_CREATED
        || diffCollidable == ecs::WAS_CREATED);

    // TODO modify only if diffPose was modified by a different system
    auto doModify = (!doDelete) && (!doCreate) && (
        diffGeometry == ecs::WAS_MODIFIED
        || diffPose == ecs::WAS_MODIFIED
        || diffCollidable == ecs::WAS_MODIFIED);

    // Another system created a new geometry we don't know about yet, so
    // create it internally
    if (doCreate)
    {
      this->CreateRigidBody(entity);
    }
    else if (doDelete)
    {
      // TODO Remove a body from the physics engine world
    }
    else if (doModify)
    {
      // TODO Update body in physics engine world
    }
  }

  // STEP 2 Update the world
  this->dynamicsWorld->stepSimulation(this->maxStepSize, 0, this->maxStepSize);

  // STEP 3 Update the simulation time
  // Physics controls simulation time because engines with a variable time step
  // will update at an unknown rate
  const double changeInTime = this->maxStepSize;
  ignition::common::Time delta(changeInTime);
  mgr.SimulationTime(mgr.SimulationTime() + delta);

  // STEP 4
  // TODO Publish contacts on an ignition transport topic?
  // If contacts are put into components, systems can't use them until the next
  // time step.

  // STEP 5 update the components
  // Components are updated via btMotionState
}


/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::PhysicsSystem,
                                  gazebo::ecs::System)
