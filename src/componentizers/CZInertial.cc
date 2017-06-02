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
#include "gazebo/components/Inertial.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "CZInertial.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZInertial::~CZInertial()
{
}

//////////////////////////////////////////////////
void CZInertial::Init()
{
  igndbg << "Registering Inertial component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::Inertial>(
      "gazebo::components::Inertial");
}

//////////////////////////////////////////////////
void CZInertial::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "inertial")
  {
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      ignwarn << "No parent on inertial, ignoring" << std::endl;
    }
    else if (parent->GetName() != "link")
    {
      ignwarn << "Parent is not a link, ignoring" << std::endl;
    }
    else
    {
      // Create the inertial component on the entity associated with the link
      ecs::EntityId parentId = _ids.at(parent.get());
      ecs::Entity &entity = _mgr.Entity(parentId);

      auto comp = entity.AddComponent<components::Inertial>();
      if (_elem.HasElement("mass"))
      {
        comp->mass = _elem.Get<double>("mass");
      }
      if (_elem.HasElement("inertia"))
      {
        sdf::ElementPtr inertia = _elem.GetElement("inertia");
        const double ixx = inertia->Get<double>("ixx");
        const double iyy = inertia->Get<double>("iyy");
        const double izz = inertia->Get<double>("izz");
        const double ixy = inertia->Get<double>("ixy");
        const double ixz = inertia->Get<double>("ixz");
        const double iyz = inertia->Get<double>("iyz");
        comp->inertia.Set(ixx, ixy, ixz,
                          ixy, iyy, iyz,
                          ixz, iyz, izz);
      }
      igndbg << "Added Inertial to " << parentId << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZInertial,
                                  gazebo::ecs::Componentizer)
