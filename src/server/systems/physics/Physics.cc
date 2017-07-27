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

#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include "gazebo/server/components/Inertial.hh"
#include "gazebo/server/components/Geometry.hh"
#include "gazebo/server/components/PhysicsConfig.hh"
#include "gazebo/server/components/Pose.hh"
#include "gazebo/server/components/WorldVelocity.hh"
#include "gazebo/server/Manager.hh"
#include "gazebo/server/EntityQuery.hh"
#include "Physics.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void Physics::Init(server::EntityQueryRegistrar &_registrar)
{
  // Query for bodies to simulate
  server::EntityQuery query;

  if (!query.AddComponent("ignition::math::Pose3d"))
    ignerr << "Undefined component[ignition::math::Pose3d]\n";

  _registrar.Register(std::move(query),
      std::bind(&Physics::UpdateBodies, this, std::placeholders::_1,
        std::placeholders::_2));
}

/////////////////////////////////////////////////
void Physics::UpdateBodies(const server::Manager *_mgr,
    const server::EntityQuery &_result)
{
  // The graph class provides Entity context, such as relative pose frames.
  // It can also be used to get extra information about simulation.
  auto &graph = _mgr->Graph();

  // Step 1 Update the world
  // \todo Step the world once

  // Step 2 Update the simulation time
  // Physics controls simulation time because engines with a variable time step
  // will update at an unknown rate
  //const double changeInTime = this->maxStepSize;
  //ignition::common::Time delta(changeInTime);
  //mgr->SimulationTime(mgr->SimulationTime() + delta);

  // Step 3
  // \todo Publish contacts on an ignition transport topic?

  // Step 4 update the components
  // \todo Update with results from the physics engine
}

/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::Physics,
                                  gazebo::server::System)
