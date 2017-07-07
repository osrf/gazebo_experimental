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

#include "components/Fraction.api.hh"
#include "gazebo/ecs/Manager.hh"
#include "systems/DivideAndPrintResult.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void DivideAndPrintResult::Init(ecs::QueryRegistrar &_registrar)
{
  ecs::EntityQuery query;

  // Add components which are required
  if (!query.AddComponent<components::Fraction>())
    std::cerr << "Undefined component[components::Fraction]\n";

  _registrar.Register(query,
      std::bind(&DivideAndPrintResult::Update, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void DivideAndPrintResult::Update(const ecs::EntityQuery &_result)
{
  ecs::Manager &mgr = this->Manager();
  auto handle = mgr.Handle();
  // Loop through all of the entities which have the required components
  for (auto const &entityId : _result.EntityIds())
  {
    auto &entity = handle->Entity(entityId);
    auto fraction = entity.Component<components::Fraction>();

    std::cout << "Dividing " << entityId << ":" <<
      fraction.Numerator() / fraction.Denominator() << std::endl;
  }
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::DivideAndPrintResult,
                                  gazebo::ecs::System)
