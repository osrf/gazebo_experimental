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

#ifndef GAZEBO_COMPONENTS_INERTIA_HH_
#define GAZEBO_COMPONENTS_INERTIA_HH_

#include <ignition/math/Matrix3.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief Describes the inertial properties of an entity.
    struct Inertial
    {
      /// \brief Mass in kilograms
      double mass = 1.0;

      /// \brief inertial matrix
      ignition::math::Matrix3d inertia = {1, 0, 0,
                                          0, 1, 0,
                                          0, 0, 1};
    };
  }
}

#endif

