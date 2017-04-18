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

// From external library
#include "dumb_physics/Body.hh"
#include "dumb_physics/World.hh"

// Internal to Gazebo
#include "gazebo/components/Inertial.hh"
#include "gazebo/components/SphereGeometry.hh"
#include "gazebo/components/WorldPose.hh"
#include "gazebo/components/WorldVelocity.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/systems/DumbPhysics.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
ecs::EntityQuery DumbPhysics::Init()
{
  ecs::EntityQuery query;

  // TODO how will systems get info that should apply to everything like
  //      gravity and solver parameters?
  this->world.Gravity({0.0, 0.0, 0.0});
  this->world.SetSize({10.0, 10.0, 10.0});

  // Add required components

  // Entities must have a sphere geometry
  if (!query.AddComponent("gazebo::components::SphereGeometry"))
    std::cerr << "Undefined component[gazebo::components::SphereGeometry]\n";

  // Entities must have a world pose
  if (!query.AddComponent("gazebo::components::WorldPose"))
    std::cerr << "Undefined component[gazebo::components::WorldPose]\n";

  // Note that we add only the required components. This system will also make
  // use of the Mass and WorldVelocity components if present, but these are
  // optional

  return std::move(query);
}

/////////////////////////////////////////////////
void DumbPhysics::Update(
    double _dt, const ecs::EntityQuery &_result, ecs::Manager &_mgr)
{
  // STEP 1 Loop through entities and update internal representation
  // This is where the effects of other systems get propagated to this one,
  // for example, if a pose is changed or a body is deleted through the GUI.
  for (auto const &entityId : _result.EntityIds())
  {
    // Get entity (should check if exists?)
    auto &entity = _mgr.Entity(entityId);

    // Body is the internal representation of an entity in this system
    auto body = this->world.BodyById(entityId);

    // Check if geometry has changed since last time step
    // We use the presence of this component to add/remove bodies in this system
    auto difference = entity.IsDifferent<components::SphereGeometry>();

    // Another system created a new geometry we don't know about yet, so
    // create it internally
    if (ecs::WAS_CREATED == difference && !body)
    {
      body = this->AddBody(entityId, entity);
    }
    // Geometry component was removed from this entity by another system.
    // We remove the whole entity from this system, because we're not
    // interested in entities without a geometry.
    else if (ecs::WAS_DELETED == difference && body)
    {
      this->world.RemoveBody(body->Id());
    }
    // Another system modified the geometry
    // TODO How can we be sure it wasn't us who changed it?
    else if (ecs::WAS_MODIFIED == difference && body)
    {
      auto const geom = entity.Component<components::SphereGeometry>();
      this->SyncInternalGeom(body, geom);
    }
    // Something went wrong
    else if (ecs::NO_DIFFERENCE != difference)
    {
      std::cerr << "Unable to handle difference [" << difference
                << "] on SphereGeometry component for entity [" << entityId
                << "]" << std::endl;
    }

    // Sync other properties in case they've been changed by other systems
    auto const pose = entity.Component<components::WorldPose>();
    if (pose)
      this->SyncInternalPose(body, pose);
    else
    {
      std::cerr << "Entity [" << entityId
                << "] missing required component WorldPose. "
                << "Removing it from the world." << std::endl;
      this->world.RemoveBody(body->Id());
    }

    auto const velocity = entity.Component<components::WorldVelocity>();
    if (velocity)
      this->SyncInternalVelocity(body, velocity);
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
    dumb_physics::Body *body = this->world.BodyById(entityId);

    if (!body)
    {
      std::cerr << "Null body for entity [" << entityId << "]" << std::endl;
      continue;
    }

    auto &entity = _mgr.Entity(entityId);

    auto worldPose = entity.ComponentMutable<components::WorldPose>();
    this->SyncExternalPose(body, worldPose);

    auto worldVel = entity.ComponentMutable<components::WorldVelocity>();
    this->SyncExternalVelocity(body, worldVel);
  }
}

/////////////////////////////////////////////////
void DumbPhysics::SyncInternalGeom(dumb_physics::Body *_body,
    const components::SphereGeometry *_component)
{
  _body->Radius(_component->radius);
}

/////////////////////////////////////////////////
void DumbPhysics::SyncInternalMass(dumb_physics::Body *_body,
    const components::Inertial *_component)
{
  _body->Mass(_component->mass);
}

/////////////////////////////////////////////////
void DumbPhysics::SyncInternalVelocity(dumb_physics::Body *_body,
    const components::WorldVelocity *_component)
{
  _body->LinearVelocity(_component->linear);
  _body->AngularVelocity(_component->angular);
}

/////////////////////////////////////////////////
void DumbPhysics::SyncInternalPose(dumb_physics::Body *_body,
    const components::WorldPose *_component)
{
  _body->Position(_component->position);
  _body->Rotation(_component->rotation);
}

/////////////////////////////////////////////////
void DumbPhysics::SyncExternalGeom(const dumb_physics::Body *_body,
    components::SphereGeometry *_component)
{
  _component->radius = _body->Radius();
}

/////////////////////////////////////////////////
void DumbPhysics::SyncExternalMass(const dumb_physics::Body *_body,
    components::Inertial *_component)
{
  _component->mass = _body->Mass();
}

/////////////////////////////////////////////////
void DumbPhysics::SyncExternalVelocity(const dumb_physics::Body *_body,
    components::WorldVelocity *_component)
{
  _component->linear = _body->LinearVelocity();
  _component->angular = _body->AngularVelocity();
}

/////////////////////////////////////////////////
void DumbPhysics::SyncExternalPose(const dumb_physics::Body *_body,
    components::WorldPose *_component)
{
  _component->position = _body->Position();
  _component->rotation = _body->Rotation();
}

/////////////////////////////////////////////////
dumb_physics::Body *DumbPhysics::AddBody(const ecs::EntityId _id,
                                         ecs::Entity &_entity)
{
  std::cout << "[phys] Add body " << _id << std::endl;

  dumb_physics::Body *body = nullptr;

  // Required components
  auto geom = _entity.Component<components::SphereGeometry>();
  auto worldPose = _entity.Component<components::WorldPose>();

  if (!geom || !worldPose)
  {
    std::cerr << "Entity [" << _entity.Id() << "] is missing required components."
             << " Can't create body." << std::endl;
    return body;
  }

  // Create body
  body = this->world.AddBody(_id);

  // Update internal representation
  this->SyncInternalGeom(body, geom);
  this->SyncInternalPose(body, worldPose);

  // Optional components
  auto inertia = _entity.Component<components::Inertial>();

  if (inertia)
    this->SyncInternalMass(body, inertia);

  auto worldVel = _entity.Component<components::WorldVelocity>();

  if (worldVel)
    this->SyncInternalVelocity(body, worldVel);

  return body;
}

/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::DumbPhysics,
                                  gazebo::ecs::System)
