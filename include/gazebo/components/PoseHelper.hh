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

#ifndef GAZEBO_COMPONENTS_POSEHELPER_HH_
#define GAZEBO_COMPONENTS_POSEHELPER_HH_

#include <ignition/math/Pose3.hh>
#include <gazebo/components/Pose.api.hh>
#include <gazebo/ecs/Manager.hh>

/// \brief This file has functions that make dealing with Pose components easier

namespace gazebo
{
  namespace components
  {
    /// \brief Get the world pose of this component
    /// \description If the Pose component on the entity is not attached to
    ///              any other, this method returns the pose unmodified. If
    ///              the component is attached to an entity who is unattached
    ///              to any other, it combines the two to return a pose
    ///              expressed in the world frame. In any other situation the
    ///              pose is not defined, and this method returns false.
    /// \param[in] _mgr a manager instance to use
    /// \param[in] _entity The entity with the pose component
    /// \param[out] _pose Object will contain pose if there are no errors
    /// \returns true iff the world pose was sucessfully calculated
    bool WorldPose(const ecs::Manager &_mgr, const ecs::Entity &_entity,
        ignition::math::Pose3d &_pose);

    /// \brief Set the world pose of this component
    /// \description If the Pose component on the entity is not attached to
    ///              any other, this method sets the pose unmodified. If
    ///              the component is attached to an entity who is unattached
    ///              to any other, it moves that entity until this one is at
    ///              the given pose, leaving the relative pose between the two
    ///              unchanged. In any other situation the pose cannot be
    ///              defined, and this method returns false.
    /// \param[in] _mgr a manager instance to use
    /// \param[in] _entity The entity with the pose component
    /// \param[in] _pose Pose in world frame that should be set
    /// \returns true iff the world pose was sucessfully set
    bool SetWorldPose(const ecs::Manager &_mgr, const ecs::Entity &_entity,
        const ignition::math::Pose3d &_pose);
  }
}

#endif
