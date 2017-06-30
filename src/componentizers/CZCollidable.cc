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
#include <gazebo/components/Collidable.api.hh>
#include "CZCollidable.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZCollidable::~CZCollidable()
{
}

//////////////////////////////////////////////////
void CZCollidable::Init()
{
}

//////////////////////////////////////////////////
void CZCollidable::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "collision")
  {
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      ignwarn << "No parent on collision, ignoring" << std::endl;
    }
    else if (parent->GetName() != "link")
    {
      ignwarn << "Parent is not a link, ignoring" << std::endl;
    }
    else
    {
      if (_elem.GetNextElement("collision")
          || parent->GetElement("collision").get() != &_elem)
      {
        // Multiple collisions on a link, TODO 
      }
      else
      {
        // Mark the link itself as collidable.
        // The collision geometry is there too
        ecs::EntityId parentId = _ids.at(parent.get());
        ecs::Entity &parentEntity = _mgr.Entity(parentId);

        auto comp = parentEntity.AddComponent<components::Collidable>();
      }

      // TODO surface properties?
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZCollidable,
                                  gazebo::ecs::Componentizer)
