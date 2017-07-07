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

#include "gazebo/components/Inertial.api.hh"
#include "gazebo/components/Geometry.api.hh"
#include "gazebo/components/Material.api.hh"
#include "gazebo/components/Pose.api.hh"
#include "gazebo/components/WorldVelocity.api.hh"
#include "gazebo/ecs/Manager.hh"


// Example of an application with 1 system:
// * Physics
int main(int argc, char **argv)
{
  // Central ECS manager
  gazebo::ecs::Manager manager;

  // Plugin loader (plugins are systems)
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");

  std::vector<std::string> componentLibs = {
    "gazeboComponentInertial",
    "gazeboComponentGeometry",
    "gazeboComponentMaterial",
    "gazeboComponentPose",
    "gazeboComponentWorldVelocity",
  };

  for (auto const &libName : componentLibs)
  {
    std::string pathToLibrary = sp.FindSharedLibrary(libName);
    if (pathToLibrary.empty())
      std::cerr << "Unable to find " << libName << std::endl;
    else
    {
      std::string pluginName = pluginLoader.LoadLibrary(pathToLibrary);
      if (pluginName.size())
      {
        std::unique_ptr<gazebo::ecs::ComponentFactory> cf;
        cf = pluginLoader.Instantiate<gazebo::ecs::ComponentFactory>(
            pluginName);
        if (!manager.LoadComponentFactory(std::move(cf)))
          std::cerr << "Failed to load " << pluginName << " from " << libName
            << std::endl;
      }
      else
        std::cerr << "Failed to load library " << libName << std::endl;
    }
  }

  std::vector<std::string> systemLibs = {
    "DumbPhysicsPlugin",
    "DummyRenderingPlugin",
  };

  for (auto const &libName : systemLibs)
  {
    std::string pathToLibrary = sp.FindSharedLibrary(libName);
    if (pathToLibrary.empty())
      std::cerr << "Unable to find " << libName << std::endl;
    else
    {
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
  }

  auto handle = manager.Handle();
  // Create 25 sphere entities
  for (int i = 0; i < 25; i++)
  {
    // Create the entity
    gazebo::ecs::EntityId e = handle->CreateEntity();
    gazebo::ecs::Entity &entity = handle->Entity(e);

    // Give it components

    // Inertial component
    auto inertial = entity.AddComponent<gazebo::components::Inertial>();
    if (inertial)
    {
      inertial.Mass() = ignition::math::Rand::DblUniform(0.1, 5.0);
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
      geom.Shape().Sphere().Radius() = ignition::math::Rand::DblUniform(0.1, 0.5);
    }
    else
    {
      std::cerr << "Failed to add geom component to entity [" << e << "]"
                << std::endl;
    }

    // Pose
    auto pose = entity.AddComponent<gazebo::components::Pose>();
    if (pose)
    {
      pose.Origin().Pos().X(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose.Origin().Pos().Y(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose.Origin().Pos().Z(ignition::math::Rand::DblUniform(-4.0, 4.0));
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
      vel.Linear().X(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel.Linear().Y(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel.Linear().Z(ignition::math::Rand::DblUniform(-1.0, 1.0));
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
      material.Appearance().Color().Red() =
        ignition::math::Rand::DblUniform(0.1, 1.0);
      material.Appearance().Color().Green() =
        ignition::math::Rand::DblUniform(0.1, 1.0);
      material.Appearance().Color().Blue() =
        ignition::math::Rand::DblUniform(0.1, 1.0);
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
