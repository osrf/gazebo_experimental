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

#include "components/Fraction.api.hh"
#include "components/Triplet.api.hh"

// Factory includes are normally unnecessary/unavailable
#include "components/Fraction.factory.hh"
#include "components/Triplet.factory.hh"

#include "gazebo/ecs/Manager.hh"

#include "systems/DivideAndPrintResult.hh"

int main(int argc, char **argv)
{
  gazebo::ecs::Manager manager;

  // Something to deal with loading plugins
  ignition::common::PluginLoader pm;

  // Normally component factories are loaded as plugins,
  // but this example has to link against them anyways to create components
  manager.LoadComponentFactory<components::FractionFactory>();
  manager.LoadComponentFactory<components::TripletFactory>();

  // First way to load a system: not using a plugin. Useful for testing
  manager.LoadSystem<gazebo::systems::DivideAndPrintResult>("DivideAndPrint");

  // Second way to load a system: using a plugin.
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");
  std::string pathToLib = sp.FindSharedLibrary("AddAndPrintResult");
  std::string pluginName;
  if (pathToLib.empty())
    std::cerr << "Failed to find AddAndPrintResult library" << std::endl;
  else
    pluginName = pm.LoadLibrary(pathToLib);

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

    if (e % 2 == 0)
    {
      auto fraction = entity.AddComponent<components::Fraction>();
      if (fraction)
      {
        fraction.Numerator() = 100.0f + i;
        fraction.Denominator() = 1.0f + i;
      }
      else
      {
        std::cerr << "Failed to add fraction to entity" << std::endl;
      }

    }
    if (e % 3 == 0)
    {
      // Another method of adding a component to an entity
      auto numbers = entity.AddComponent<components::Triplet>();
      if (numbers)
      {
        numbers.First() = e;
        numbers.Second() = i;
        numbers.Third() = 3;
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
