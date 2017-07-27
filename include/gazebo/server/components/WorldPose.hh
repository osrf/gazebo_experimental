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

#ifndef GAZEBO_COMPONENTS_WORLDPOSE_HH_
#define GAZEBO_COMPONENTS_WORLDPOSE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief Pose of an object
    /// \deprecated use Pose.hh instead
    ///
    /// This is special in that is always in world frame. For transforming
    /// poses between different frames the pose graph should be used
    struct WorldPose
    {
      /// \brief position in world frame (meters)
      ignition::math::Vector3<double> position = {0, 0, 0};
      /// \brief rotation in world frame
      ignition::math::Quaternion<double> rotation = {1, 0, 0, 0};
    };
  }
}

#endif

