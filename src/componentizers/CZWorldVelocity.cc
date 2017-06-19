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
#include <gazebo/components/WorldVelocity.api.hh>
#include "CZWorldVelocity.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZWorldVelocity::~CZWorldVelocity()
{
}

//////////////////////////////////////////////////
void CZWorldVelocity::Init()
{
}

//////////////////////////////////////////////////
void CZWorldVelocity::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "link")
  {
    // Create the WorldVelociy component on the entity associated with the link
    // as long as the model is not static
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      ignwarn << "Link with no parent, ignoring" << std::endl;
    }
    else if (parent->GetName() != "model")
    {
      ignwarn << "<link> is not part of a model, ignoring" << std::endl;
    }
    else
    {
      if (parent->HasElement("static") && parent->Get<bool>("static"))
      {
        // Static, No need for velocity
        return;
      }

      ecs::EntityId id = _ids.at(&_elem);
      ecs::Entity &entity = _mgr.Entity(id);
      entity.AddComponent<components::WorldVelocity>();
      igndbg << "Added WorldVelocity to " << id << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZWorldVelocity,
                                  gazebo::ecs::Componentizer)
