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
#include <cstdlib>

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>

#include "components/Fraction.hh"
#include "components/Triplet.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"

#include "systems/DivideAndPrintResult.hh"

int main(int argc, char **argv)
{
  gazebo::ecs::Manager manager;

  // Something to deal with loading plugins
  ignition::common::PluginLoader pm;

  // TODO Componentizer to register components
  gazebo::ecs::ComponentFactory::Register<gazebo::components::Triplet>(
      "gazebo::components::Triplet");
  gazebo::ecs::ComponentFactory::Register<gazebo::components::Fraction>(
      "gazebo::components::Fraction");

  // First way to load a system: not using a plugin. Useful for testing
  manager.LoadSystem<gazebo::systems::DivideAndPrintResult>("DivideAndPrint");

  // Second way to load a system: using a plugin.
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");
  std::string pathToLib = sp.FindSharedLibrary("AddAndPrintResult");
  std::string pluginName = pm.LoadLibrary(pathToLib);

  if (pluginName.size())
  {
    std::unique_ptr<gazebo::ecs::System> sys;
    sys = pm.Instantiate<gazebo::ecs::System>(pluginName);
    if (!manager.LoadSystem(pluginName, std::move(sys)))
    {
      std::cerr << "Failed to load plugin from library" << std::endl;
    }
  }
  else
  {
    std::cerr << "Failed to load library" << std::endl;
  }

  // Create a few entities to work with
  for (int i = 0; i < 10; i++)
  {
    // An entity is just an ID
    gazebo::ecs::EntityId e = manager.CreateEntity();
    // Convenience wrapper for working with an Id
    gazebo::ecs::Entity &entity = manager.Entity(e);

    // TODO manager.CreateEntity<ComponentA, ComponentB, ...>();

    if (e % 2 == 0)
    {
      auto *fraction = entity.AddComponent<gazebo::components::Fraction>();
      if (nullptr != fraction)
      {
        fraction->numerator = 100.0f + i;
        fraction->denominator = 1.0f + i;
      }
      else
      {
        std::cerr << "Failed to add fraction to entity" << std::endl;
      }

    }
    if (e % 3 == 0)
    {
      // Another method of adding a component to an entity
      auto numbers = entity.AddComponent<gazebo::components::Triplet>();
      if (nullptr != numbers)
      {
        numbers->first = e;
        numbers->second = i;
        numbers->third = 3;
      }
      else
      {
        std::cerr << "Failed to add triplet to entity" << std::endl;
      }
    }
  }

  // Run all the systems once.
  manager.UpdateOnce();

  return 0;
}
