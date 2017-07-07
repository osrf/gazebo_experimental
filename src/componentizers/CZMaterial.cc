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
#include <gazebo/components/Material.api.hh>
#include "CZMaterial.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZMaterial::~CZMaterial()
{
}

//////////////////////////////////////////////////
void CZMaterial::Init()
{
}

//////////////////////////////////////////////////
void CZMaterial::FromSDF(std::unique_ptr<ecs::DataHandle> _handle,
    sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "visual")
  {
    // If there's a material element, wait for that one
    if (_elem.GetElement("material"))
      return;

    // No material, use the default material
    ecs::EntityId id = _ids.at(&_elem);
    ecs::Entity &visualEntity = _handle->Entity(id);
    auto material = visualEntity.AddComponent<components::Material>();
    material.Appearance().Color().Red() = 0.7;
    material.Appearance().Color().Green() = 0.7;
    material.Appearance().Color().Blue() = 0.7;
    material.Appearance().Color().Alpha() = 1.0;
    igndbg << "Added default material to " << id << std::endl;
  }
  else if (_elem.GetName() == "material")
  {
    // Group the material component with other components on the parent
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      ignwarn << "No parent of material" << std::endl;
    }
    else if (parent->GetName() != "visual")
    {
      ignwarn << "Parent must be <visual>, not " << parent->GetName()
        << std::endl;
    }
    else if (!_elem.GetElement("ambient"))
      ignwarn << "Only support flat color materials given with <ambient>"
        << std::endl;
    else
    {
      ecs::EntityId id = _ids.at(parent.get());
      ecs::Entity &visualEntity = _handle->Entity(id);
      sdf::ElementPtr ambient = _elem.GetElement("ambient");
      auto color = ambient->Get<ignition::math::Color>();

      auto material = visualEntity.AddComponent<components::Material>();
      material.Appearance().Color().Red() = color[0];
      material.Appearance().Color().Green() = color[1];
      material.Appearance().Color().Blue() = color[2];
      material.Appearance().Color().Alpha() = color[3];
      igndbg << "Added flat color material to " << id << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZMaterial,
                                  gazebo::ecs::Componentizer)
