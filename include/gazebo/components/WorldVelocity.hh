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

#ifndef GAZEBO_COMPONENTS_WORLDVELOCITY_HH_
#define GAZEBO_COMPONENTS_WORLDVELOCITY_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief Velocity of an object in world frame
    struct WorldVelocity
    {
      /// \brief Linear velocity (m/s)
      ignition::math::Vector3<double> linear = {0, 0, 0};
      /// \brief Angular velocity (rad/s)
      ignition::math::Quaternion<double> angular = {1, 0, 0, 0};
    };
  }
}

#endif


