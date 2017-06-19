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

#include <gazebo/components/Inertial.api.hh>
#include <gazebo/components/Geometry.api.hh>
#include <gazebo/components/PhysicsConfig.api.hh>
#include <gazebo/components/Pose.api.hh>
#include <gazebo/components/WorldVelocity.api.hh>
#include <gazebo/ecs/Manager.hh>
#include <gazebo/ecs/EntityQuery.hh>
#include "PhysicsSystem.hh"

namespace gzsys = gazebo::systems;
using namespace gzsys;
using namespace gazebo;


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
    std::cerr << "Undefined component[gazebo::components::Geometry]\n";
  if (!query.AddComponent<gazebo::components::Pose>())
    std::cerr << "Undefined component[gazebo::components::Pose]\n";
  // TODO require component with contact or surface properies

  // Note that we add only the required components. This system will also make
  // use of the Mass and WorldVelocity components if present, but these are
  // optional

  _registrar.Register(query,
      std::bind(&PhysicsSystem::UpdateBodies, this, std::placeholders::_1));
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

    auto doDelete = (diffGeometry == ecs::WAS_DELETED)
      || (diffPose == ecs::WAS_DELETED);
    auto doCreate = (!doDelete) && (diffGeometry == ecs::WAS_CREATED
        || diffPose == ecs::WAS_CREATED);
    auto doModify = (!doDelete) && (!doCreate) && (
        diffGeometry == ecs::WAS_CREATED || diffPose == ecs::WAS_CREATED);

    // Another system created a new geometry we don't know about yet, so
    // create it internally
    if (doCreate)
    {
      // TODO Add a new body to the physics engine world
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
  // TODO Step the world once

  // STEP 3 Update the simulation time
  // Physics controls simulation time because engines with a variable time step
  // will update at an unknown rate
  const double changeInTime = this->maxStepSize;
  ignition::common::Time delta(changeInTime);
  mgr.SimulationTime(mgr.SimulationTime() + delta);

  // STEP 4
  // TODO Publish contacts on an ignition transport topic?

  // STEP 5 update the components
  // TODO Update with results from the physics engine
}


/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::PhysicsSystem,
                                  gazebo::ecs::System)
