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

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <ignition/common/PluginLoader.hh>
#include <ignition/math/Rand.hh>

#include "gazebo/components/RigidBody.hh"
#include "gazebo/components/WorldPose.hh"
#include "gazebo/components/WorldVelocity.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"


int main(int argc, char **argv)
{
  gazebo::ecs::Manager manager;

  // TODO Componentizer to register components
  gazebo::ecs::ComponentFactory::Register<gazebo::components::RigidBody>(
      "gazebo::components::RigidBody");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::WorldPose>(
      "gazebo::components::WorldPose");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::WorldVelocity>(
      "gazebo::components::WorldVelocity");

  ignition::common::PluginLoader pm;
  const char *path = std::getenv("GAZEBO_PLUGIN_PATH");
  if (nullptr != path)
    pm.AddSearchPath(path);
  else
    std::cerr << "No plugin path given" << std::endl;

  if (pm.LoadLibrary("DumbPhysicsPlugin"))
  {
    std::unique_ptr<gazebo::ecs::System> sys;
    sys = pm.Instantiate<gazebo::ecs::System>(
        "::gazebo::systems::DumbPhysics");
    if (!manager.LoadSystem(std::move(sys)))
    {
      std::cerr << "Failed to load plugin from library" << std::endl;
    }
  }
  else
  {
    std::cerr << "Failed to load library" << std::endl;
  }

  // Add 25 spheres
  for (int i = 0; i < 25; i++)
  {
    gazebo::ecs::EntityId e = manager.CreateEntity();
    auto body = manager.AddComponent<gazebo::components::RigidBody>(e);
    auto pose = manager.AddComponent<gazebo::components::WorldPose>(e);
    auto vel = manager.AddComponent<gazebo::components::WorldVelocity>(e);
    if (body && pose && vel)
    {
      body->type = gazebo::components::RigidBody::SPHERE;
      body->isStatic = false;
      body->mass = ignition::math::Rand::DblUniform(0.1, 5.0);
      body->sphere.radius = ignition::math::Rand::DblUniform(0.1, 0.5);

      pose->position.X(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Y(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Z(ignition::math::Rand::DblUniform(-4.0, 4.0));

      vel->linear.X(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Y(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Z(ignition::math::Rand::DblUniform(-1.0, 1.0));
    }
    else
    {
      std::cerr << "Failed to add components to entity" << std::endl;
    }
  }

  int millis = 1;
  double timeStep = millis / 1000.0;
  while (true)
  {
    manager.UpdateSystems(timeStep);
    // update in actual time
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
  }

  return 0;
}
