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
#include "gazebo/components/Material.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "CZMaterial.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
void CZMaterial::Init()
{
  igndbg << "Registering Material component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::Material>(
      "gazebo::components::Material");
}

//////////////////////////////////////////////////
void CZMaterial::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "visual")
  {
    // If there's a material element, wait for that one
    if (_elem.GetElement("material"))
      return;

    // No material, use the default material
    ecs::EntityId id = _ids.at(&_elem);
    ecs::Entity &visualEntity = _mgr.Entity(id);
    auto material = visualEntity.AddComponent<components::Material>();
    material->type = components::Material::COLOR;
    material->color.red = 0.7;
    material->color.green = 0.7;
    material->color.blue = 0.7;
    material->color.alpha = 1.0;
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
      ignwarn << "Only support flat color materials" << std::endl;
    else
    {
      ecs::EntityId id = _ids.at(parent.get());
      ecs::Entity &visualEntity = _mgr.Entity(id);
      sdf::ElementPtr ambient = _elem.GetElement("ambient");
      auto color = ambient->Get<ignition::math::Vector4d>();

      auto material = visualEntity.AddComponent<components::Material>();
      material->type = components::Material::COLOR;
      material->color.red = color[0];
      material->color.green = color[1];
      material->color.blue = color[2];
      material->color.alpha = color[3];
    igndbg << "Added flat color material to " << id << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZMaterial,
                                  gazebo::ecs::Componentizer)
