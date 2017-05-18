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
#include <ignition/common/SystemPaths.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Rand.hh>

#include "gazebo/components/Inertial.hh"
#include "gazebo/components/Geometry.hh"
#include "gazebo/components/Material.hh"
#include "gazebo/components/WorldPose.hh"
#include "gazebo/components/WorldVelocity.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"

// Example of an application with 1 system:
// * Physics
int main(int argc, char **argv)
{
  // Central ECS manager
  gazebo::ecs::Manager manager;

  // TODO Componentizer to register components

  // Register component types
  gazebo::ecs::ComponentFactory::Register<gazebo::components::Inertial>(
      "gazebo::components::Inertial");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::Geometry>(
      "gazebo::components::Geometry");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::WorldPose>(
      "gazebo::components::WorldPose");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::WorldVelocity>(
      "gazebo::components::WorldVelocity");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::Material>(
      "gazebo::components::Material");

  // Plugin loader (plugins are systems)
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");

  std::vector<std::string> libs = {
    "DumbPhysicsPlugin",
    "DummyRenderingPlugin",
  };

  for (auto const &libName : libs)
  {
    std::string pathToLibrary = sp.FindSharedLibrary(libName);
    std::string pluginName = pluginLoader.LoadLibrary(pathToLibrary);
    if (pluginName.size())
    {
      std::unique_ptr<gazebo::ecs::System> sys;
      sys = pluginLoader.Instantiate<gazebo::ecs::System>(pluginName);
      if (!manager.LoadSystem(pluginName, std::move(sys)))
        std::cerr << "Failed to load " << pluginName << " from " << libName
          << std::endl;
    }
    else
      std::cerr << "Failed to load library " << libName << std::endl;
  }

  // Create 25 sphere entities
  for (int i = 0; i < 25; i++)
  {
    // Create the entity
    gazebo::ecs::EntityId e = manager.CreateEntity();
    gazebo::ecs::Entity &entity = manager.Entity(e);

    // Give it components

    // Inertial component
    auto inertial = entity.AddComponent<gazebo::components::Inertial>();
    if (inertial)
    {
      inertial->mass = ignition::math::Rand::DblUniform(0.1, 5.0);
    }
    else
    {
      std::cerr << "Failed to add inertial component to entity [" << e << "]"
                << std::endl;
    }

    // Geometry component
    auto geom = entity.AddComponent<gazebo::components::Geometry>();
    if (geom)
    {
      geom->type = gazebo::components::Geometry::SPHERE;
      geom->sphere.radius = ignition::math::Rand::DblUniform(0.1, 0.5);
    }
    else
    {
      std::cerr << "Failed to add geom component to entity [" << e << "]"
                << std::endl;
    }

    // World pose
    auto pose = entity.AddComponent<gazebo::components::WorldPose>();
    if (pose)
    {
      pose->position.X(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Y(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Z(ignition::math::Rand::DblUniform(-4.0, 4.0));
    }
    else
    {
      std::cerr << "Failed to add world pose component to entity [" << e << "]"
                << std::endl;
    }

    // World velocity
    auto vel = entity.AddComponent<gazebo::components::WorldVelocity>();
    if (vel)
    {
      vel->linear.X(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Y(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Z(ignition::math::Rand::DblUniform(-1.0, 1.0));
    }
    else
    {
      std::cerr << "Failed to add world velocity component to entity ["
                << e << "]" << std::endl;
    }

    // Renderable
    auto material = entity.AddComponent<gazebo::components::Material>();
    if (material)
    {
      material->type = gazebo::components::Material::COLOR;
      material->color.red = ignition::math::Rand::DblUniform(0.1, 1.0);
      material->color.green = ignition::math::Rand::DblUniform(0.1, 1.0);
      material->color.blue = ignition::math::Rand::DblUniform(0.1, 1.0);
    }
    else
    {
      std::cerr << "Failed to add Material component to entity ["
                << e << "]" << std::endl;
    }
  }

  // Simulation loop
  const double real_time_factor = 1.0;
  while (true)
  {
    manager.UpdateOnce(real_time_factor);
  }

  return 0;
}
