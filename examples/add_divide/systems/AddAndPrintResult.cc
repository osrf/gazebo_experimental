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

#include "components/Triplet.hh"
#include "gazebo/server/Manager.hh"
#include "systems/AddAndPrintResult.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void AddAndPrintResult::Init(server::EntityQueryRegistrar &_registrar)
{
  server::EntityQuery query;

  // Add components which are required
  if (!query.AddComponent("gazebo::components::Triplet"))
    std::cerr << "Undefined component[gazebo::components::Triplet]\n";

  _registrar.Register(std::move(query),
      std::bind(&AddAndPrintResult::Update, this, std::placeholders::_1,
        std::placeholders::_2));
}

/////////////////////////////////////////////////
void AddAndPrintResult::Update(const server::Manager *_mgr,
    const server::EntityQuery &_query)
{
  // Loop through all of the entities which have the required components
  for (auto const &entityId : _query.EntityIds())
  {
    auto &entity = _query.EntityById(entityId);
    auto const *numbers = entity.Component<gazebo::components::Triplet>();
    if (numbers)
    {
      std::cout << "Adding " << entityId << ":" <<
        numbers->first + numbers->second + numbers->third << std::endl;
    }
    else
      std::cerr << "Invalid numbers\n";
  }
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::AddAndPrintResult,
                          gazebo::server::System)
