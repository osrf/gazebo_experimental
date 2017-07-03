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

#include <ignition/common/Console.hh>
#include <gazebo/components/PoseHelper.hh>

namespace gzcomp = gazebo::components;
using namespace gazebo;
using namespace gzcomp;

//////////////////////////////////////////////////
bool gzcomp::WorldPose(const ecs::Manager &_mgr, const ecs::Entity &_entity,
    ignition::math::Pose3d &_pose)
{
  bool success = true;
  auto pose = _entity.Component<components::Pose>();

  if (!pose)
  {
    ignerr << "No Pose on [" << _entity.Id() << "]\n";
    success = false;
  }
  else if (pose.AttachedTo() == ecs::NO_ENTITY)
  {
    // Pose is in world frame
    _pose = pose.Transform();
  }
  else
  {
    // Pose is relative to another
    auto &entity = _mgr.Entity(pose.AttachedTo());
    if (entity.Id() == ecs::NO_ENTITY)
    {
      ignerr << "Pose on [" << _entity.Id()
        << "] is relative to a nonexistant entity [" << entity.Id() << "]\n";
      success = false;
    }
    else
    {
      auto basePose = entity.Component<gazebo::components::Pose>();
      if (!basePose)
      {
        ignerr << "Pose on [" << _entity.Id()
          << "] is relative to entity [" << entity.Id() << "] with no pose\n";
        success = false;
      }
      else if (basePose.AttachedTo() != ecs::NO_ENTITY)
      {
        ignerr << "Pose on [" << _entity.Id()
          << "] is relative to an entity [" << entity.Id()
          << "] with a relative pose\n";
        success = false;
      }
      else
      {
        _pose = pose.Transform() + basePose.Transform();
      }
    }
  }
  return success;
}

//////////////////////////////////////////////////
bool gzcomp::SetWorldPose(const ecs::Manager &_mgr, const ecs::Entity &_entity,
    const ignition::math::Pose3d &_pose)
{
  bool success = true;

  auto pose = _entity.ComponentMutable<components::Pose>();

  if (!pose)
  {
    ignerr << "No Pose on [" << _entity.Id() << "]\n";
    success = false;
  }
  else if (pose.AttachedTo() == ecs::NO_ENTITY)
  {
    // Pose is in world frame
    pose.Transform() = _pose;
  }
  else
  {
    // Pose is relative to another
    auto &entity = _mgr.Entity(pose.AttachedTo());
    if (entity.Id() == ecs::NO_ENTITY)
    {
      ignerr << "Pose on [" << _entity.Id()
        << "] is relative to a nonexistant entity [" << entity.Id() << "]\n";
      success = false;
    }
    else
    {
      auto basePose = entity.ComponentMutable<gazebo::components::Pose>();
      if (!basePose)
      {
        ignerr << "Pose on [" << _entity.Id()
          << "] is relative to entity [" << entity.Id() << "] with no pose\n";
        success = false;
      }
      else if (basePose.AttachedTo() != ecs::NO_ENTITY)
      {
        ignerr << "Pose on [" << _entity.Id()
          << "] is relative to an entity [" << entity.Id()
          << "] with a relative pose\n";
        success = false;
      }
      else
      {
        basePose.Transform() = pose.Transform().Inverse() + _pose;
      }
    }
  }
  return success;
}

