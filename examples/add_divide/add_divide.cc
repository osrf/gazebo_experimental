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
#include <gazebo/server/Manager.hh>

#include "components/Fraction.hh"
#include "components/Triplet.hh"
#include "systems/DivideAndPrintResult.hh"

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  ignition::common::Console::SetVerbosity(4);

  gazebo::server::Manager manager;

  // Load a system using a plugin.
  if (!manager.LoadSystem("AddAndPrintResult"))
  {
    ignerr << "Unable to load AddAndPrint system\n";
  }

  // Create a few entities to work with
  for (int i = 0; i < 10; i++)
  {
    // Create an entity
    gazebo::server::Entity &entity = manager.CreateEntity(std::to_string(i));

    if (entity.Id() % 2 == 0)
    {
      auto *fraction = entity.AddComponent<gazebo::components::Fraction>();
      if (fraction != nullptr)
      {
        fraction->numerator = 100.0f + i;
        fraction->denominator = 1.0f + i;
        std::cout << "Entity[" << entity.Id() << "] has a Fraction: "
          << "Numerator= " << fraction->numerator << " Denominator="
          << fraction->denominator << "\n";
      }
      else
      {
        ignerr << "Failed to add fraction to entity" << std::endl;
      }

    }

    if (entity.Id() % 3 == 0)
    {
      // Another method of adding a component to an entity
      auto numbers = entity.AddComponent<gazebo::components::Triplet>();
      if (numbers != nullptr)
      {
        numbers->first = entity.Id();
        numbers->second = i;
        numbers->third = 3;

        std::cout << "Entity[" << entity.Id() << "] has a Triplet: "
          << "First= " << numbers->first << " Second="
          << numbers->second << " Third=" << numbers->third << "\n";
      }
      else
      {
        ignerr << "Failed to add triplet to entity" << std::endl;
      }
    }
  }

  // Load another system.
  if (!manager.LoadSystem("DivideAndPrintResult"))
  {
    ignerr << "Unable to load DivideAndPrint system\n";
  }

  // Run all the systems once.
  manager.UpdateOnce();

  return 0;
}
