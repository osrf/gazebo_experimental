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
#include <ignition/common/Time.hh>

#include "gazebo/components/TimeInfo.hh"
#include "gazebo/ecs/ComponentFactory.hh"

#include "CZTimeInfo.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZTimeInfo::~CZTimeInfo()
{
}

//////////////////////////////////////////////////
void CZTimeInfo::Init()
{
  igndbg << "Registering TimeInfo component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::TimeInfo>(
      "gazebo::components::TimeInfo");
}

//////////////////////////////////////////////////
void CZTimeInfo::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  // Add time info component to a single entity, which represents the world
  if (_elem.GetName() == "physics")
  {
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      ignwarn << "No parent of <physics>" << std::endl;
    }
    else if (parent->GetName() != "world")
    {
      ignwarn << "Parent must be <world>, not " << parent->GetName()
        << std::endl;
    }
    else
    {
      // Entity
      ecs::EntityId id = _ids.at(&_elem);
      ecs::Entity &entity = _mgr.Entity(id);

      // Add component
      auto comp = entity.AddComponent<components::TimeInfo>();

      igndbg << "Added TimeInfo to " << id << std::endl;
    }
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZTimeInfo,
                                  gazebo::ecs::Componentizer)
