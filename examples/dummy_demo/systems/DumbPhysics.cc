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
#include <ignition/common/PluginMacros.hh>

#include "dumb_physics/Body.hh"
#include "dumb_physics/World.hh"

#include "DumbPhysics.hh"

#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/EntityQuery.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
ecs::EntityQuery DumbPhysics::Init()
{
  ecs::EntityQuery query;

  // TODO how will systems get info that should apply to everything like
  //      gravity and solver parameters?
  this->world.Gravity({0.0, 0.0, 0.0});
  this->world.SetSize(10.0, 10.0, 10.0);

  // Add required components
  if (!query.AddComponent("gazebo::components::RigidBody"))
    std::cerr << "Undefined component[gazebo::components::RigidBody]\n";
  if (!query.AddComponent("gazebo::components::WorldPose"))
    std::cerr << "Undefined component[gazebo::components::WorldPose]\n";
  if (!query.AddComponent("gazebo::components::WorldVelocity"))
    std::cerr << "Undefined component[gazebo::components::WorldVelocity]\n";

  return std::move(query);
}

/////////////////////////////////////////////////
void DumbPhysics::Update(
    double _dt, const ecs::EntityQuery &_result, ecs::Manager &_mgr)
{
  // STEP 1 Loop through entities and update internal representation
  for (auto const &entityId : _result.EntityIds())
  {
    auto &entity = _mgr.Entity(entityId);
    auto const rigidBody = entity.Component<components::RigidBody>();
    auto difference = entity.IsDifferent<components::RigidBody>();

    dumb_physics::Body *body = nullptr;

    if (ecs::WAS_CREATED == difference)
    {
      // Create a dumb_physics::Body if this is new
      auto worldPose = entity.Component<components::WorldPose>();
      body = this->AddBody(entityId, rigidBody, worldPose);
    }
    else
    {
      body = this->world.GetById(entityId);

      if (ecs::WAS_DELETED == difference)
        this->world.RemoveBody(body->Id());

      else if (ecs::WAS_MODIFIED == difference)
        this->SyncBodies(body, rigidBody);
    }

    if (body && !body->IsStatic())
    {
      auto const velocity = entity.Component<components::WorldVelocity>();
      if (velocity)
      {
        this->SyncVelocity(body, velocity);
      }
    }
  }

  // STEP 2 do some physics
  auto contacts = this->world.Update(_dt);

  // TODO Publish contacts on an ignition transport topic?
  for (auto contact : contacts)
  {
    std::cout<< "[phys]Contact " << contact.first << " and "
      << contact.second << std::endl;
  }

  // STEP 3 update the components with the results of the physics
  for (auto const &entityId : _result.EntityIds())
  {
    dumb_physics::Body *body = this->world.GetById(entityId);

    if (body)
    {
      auto &entity = _mgr.Entity(entityId);
      // TODO what if something else moved the world pose? How to know that
      //      it changed and the physics world should be updated?
      auto worldPose = entity.ComponentMutable<components::WorldPose>();
      worldPose->position = body->Position();
      worldPose->rotation = body->Rotation();

      auto component = entity.ComponentMutable<components::WorldVelocity>();
      component->linear = body->LinearVelocity();
      component->angular = body->AngularVelocity();
    }
  }
}

/////////////////////////////////////////////////
void DumbPhysics::SyncBodies(dumb_physics::Body *body,
    const components::RigidBody *component)
{
  if (component->type != components::RigidBody::SPHERE)
  {
    // Dumb physics only supports spheres
    this->world.RemoveBody(body->Id());
  }
  else
  {
    // set internal representation to match component
    body->Radius(component->sphere.radius);
    body->IsStatic(component->isStatic);
    body->Mass(component->mass);
  }
}

/////////////////////////////////////////////////
void DumbPhysics::SyncVelocity(dumb_physics::Body *body,
    const components::WorldVelocity *component)
{
    body->LinearVelocity(component->linear);
    body->AngularVelocity(component->angular);
}

/////////////////////////////////////////////////
dumb_physics::Body *DumbPhysics::AddBody(ecs::EntityId _id,
    const components::RigidBody *bodyComponent,
    const components::WorldPose *poseComponent)
{
  std::cout << "[phys]Add body " << _id << std::endl;
  dumb_physics::Body *body = nullptr;
  if (bodyComponent->type == components::RigidBody::SPHERE)
  {
    // Dumb physics only supports spheres
    body = this->world.AddBody(_id);
    body->Radius(bodyComponent->sphere.radius);
    body->IsStatic(bodyComponent->isStatic);
    body->Mass(bodyComponent->mass);

    body->Position(poseComponent->position);
    body->Rotation(poseComponent->rotation);
  }
  return body;
}

/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::DumbPhysics,
                                  gazebo::ecs::System)
