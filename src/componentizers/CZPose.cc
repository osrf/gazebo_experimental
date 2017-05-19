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
#include "gazebo/components/Pose.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "CZPose.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
void CZPose::Init()
{
  igndbg << "Registering Pose component" << std::endl;
  ecs::ComponentFactory::Register<gazebo::components::Pose>(
      "gazebo::components::Pose");
}

//////////////////////////////////////////////////
void CZPose::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  // Create frame names according to gazebo 9 document
  //
  // Give things poses and unique frame ids
  // Model - child of world: pose in frame "/"
  //       - child of another model: pose in parent model frame
  // Link - pose in model "[parent model]/model/modelName"
  // Collision - pose in link "[parent model]/model/modelName/link/linkName"
  // Visual - pose in link "[parent model]/model/modelName/link/linkName"

  std::string tag = _elem.GetName();

  if (tag == "model" || tag == "link" || tag == "visual" || tag == "collision")
  {
    // Figure out parent frame
    std::string parentFrame = "/";
    sdf::ElementPtr parent = _elem.GetParent();
    if (parent && parent->GetName() != "world")
    {
      parentFrame = this->frames.at(parent.get());
    }
    assert(!parentFrame.empty());


    // Figure out what frame this defines
    std::string definesFrame;
    std::string name = _elem.GetAttribute("name")->GetAsString();
    if (parentFrame[parentFrame.size()-1] == '/')
      definesFrame = parentFrame + tag + '/' + name;
    else
      definesFrame = parentFrame + '/' + tag + '/' + name;

    // Remember the frame this element defines
    this->frames[&_elem] = definesFrame;

    //figure out the pose
    ignition::math::Pose3d pose;
    if (_elem.HasElement("pose"))
    {
      pose = _elem.Get<ignition::math::Pose3d>("pose");
    }

    // create component
    ecs::EntityId id = _ids.at(&_elem);
    ecs::Entity &entity = _mgr.Entity(id);
    auto comp = entity.AddComponent<components::Pose>();
    comp->parentFrame = parentFrame;
    comp->definesFrame = definesFrame;
    comp->pose = pose;
    igndbg << "Pose " << pose << " in frame " << parentFrame << " on "
      << id << std::endl;
  }
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZPose,
                                  gazebo::ecs::Componentizer)
