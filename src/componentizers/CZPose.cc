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
#include <gazebo/components/Pose.api.hh>
#include "CZPose.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZPose::~CZPose()
{
}

//////////////////////////////////////////////////
void CZPose::Init()
{
}

//////////////////////////////////////////////////
void CZPose::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  std::string tag = _elem.GetName();

  // Links are attached to NO_ENTITY

  if (tag != "model" && tag != "link" && tag != "inertial" && tag != "visual"
      && tag != "collision")
  {
    // Other tags are unsupported
    return;
  }

  ecs::EntityId id = _ids.at(&_elem);
  sdf::ElementPtr parent = _elem.GetParent();
  ecs::EntityId parentId = _ids.at(parent.get());
  ecs::EntityId attachedTo = ecs::NO_ENTITY;
  ignition::math::Pose3d transform;
  if (_elem.HasElement("pose"))
  {
    transform = _elem.Get<ignition::math::Pose3d>("pose");
  }

  if (tag == "link")
  {
    if (!_elem.GetNextElement("link")
        && parent->GetElement("link").get() == &_elem)
    {
      // Cannonical link, attach model to it
      ecs::Entity &entity = _mgr.Entity(parentId);
      auto comp = entity.AddComponent<components::Pose>();
      assert(comp);
      comp.AttachedTo() = id;
      // Set the model's pose relative to the cannonical link
      comp.Transform() = transform.Inverse();
    }

    // Set the Link pose in world frame
    transform = transform + this->transforms[parentId];
  }
  else if (tag == "visual" || tag == "inertial" || tag == "collision")
  {
    // Collisions, visuals, inertials are attached to the link
    attachedTo = parentId;
  }

  this->transforms[id] = transform;

  ecs::Entity &entity = _mgr.Entity(id);
  auto comp = entity.AddComponent<components::Pose>();
  comp.AttachedTo() = attachedTo;
  comp.Transform() = transform;

  igndbg << "Added pose to " << id << " attached to " << comp.AttachedTo()
    << " transform " << comp.Transform() << std::endl;
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZPose,
                                  gazebo::ecs::Componentizer)
