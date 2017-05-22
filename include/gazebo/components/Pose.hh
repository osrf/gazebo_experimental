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

#ifndef GAZEBO_COMPONENTS_POSE_HH_
#define GAZEBO_COMPONENTS_POSE_HH_

#include <ignition/math/Pose3.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief Pose of an object
    struct Pose
    {
      /// \brief What frame is this pose defined in
      /// \remarks "" is invalid
      /// \remarks "/" means it's defined in the world frame
      std::string parentFrame = "/";

      /// \brief Name of the frame cooresponding to this pose
      /// \remarks "" means this does not define a frame
      std::string definesFrame = "";

      /// \brief pose in parent frame
      ignition::math::Pose3d pose;
    };
  }
}

#endif
