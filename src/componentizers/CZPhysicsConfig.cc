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
#include <sdf/sdf.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/math.hh>
#include "gazebo/components/PhysicsConfig.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "CZPhysicsConfig.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
void CZPhysicsConfig::Init()
{
  igndbg << "Registering PhysicsConfig component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::PhysicsConfig>(
      "gazebo::components::PhysicsConfig");
}

//////////////////////////////////////////////////
void CZPhysicsConfig::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "physics")
  {
    // Create the inertial component on the entity associated with the link
    ecs::EntityId id = _ids.at(&_elem);
    ecs::Entity &entity = _mgr.Entity(id);

    auto comp = entity.AddComponent<components::PhysicsConfig>();
    comp->maxStepSize = _elem.Get<double>("max_step_size");
    igndbg << "Added PhysicsConfig to " << id << std::endl;
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZPhysicsConfig,
                                  gazebo::ecs::Componentizer)
