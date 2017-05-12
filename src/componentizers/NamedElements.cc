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
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/Console.hh>
#include "gazebo/components/Name.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "NamedElements.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
void NamedElements::Init()
{
  igndbg << "NamedElements registering Name component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::Name>(
      "gazebo::components::Name");
}

//////////////////////////////////////////////////
void NamedElements::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    ecs::EntityId _id)
{
  // Add name component for this element
  if (_elem.HasAttribute("name"))
  {
    std::string name = _elem.GetAttribute("name")->GetAsString();
    if (!name.empty())
    {
      igndbg << "creating component named " << name << std::endl;
      ecs::Entity &e = _mgr.Entity(_id);
      auto nameComponent = e.AddComponent<components::Name>();
      nameComponent->name = name;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::NamedElements,
                                  gazebo::ecs::Componentizer)
