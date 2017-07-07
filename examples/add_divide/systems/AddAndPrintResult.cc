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

#include "components/Triplet.api.hh"
#include "gazebo/ecs/Manager.hh"
#include "systems/AddAndPrintResult.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void AddAndPrintResult::Init(ecs::QueryRegistrar &_registrar)
{
  ecs::EntityQuery query;

  // Add components which are required
  if (!query.AddComponent<components::Triplet>())
    std::cerr << "Undefined component[components::Triplet]\n";

  _registrar.Register(query,
      std::bind(&AddAndPrintResult::Update, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void AddAndPrintResult::Update(const ecs::EntityQuery &_query)
{
  ecs::Manager &mgr = this->Manager();
  auto handle = mgr.Handle();
  // Loop through all of the entities which have the required components
  for (auto const &entityId : _query.EntityIds())
  {
    auto &entity = handle->Entity(entityId);
    auto numbers = entity.Component<components::Triplet>();

    std::cout << "Adding " << entityId << ":" <<
      numbers.First() + numbers.Second() + numbers.Third() << std::endl;
  }
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::AddAndPrintResult,
                          gazebo::ecs::System)
