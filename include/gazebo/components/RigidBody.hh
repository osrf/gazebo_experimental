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

#ifndef GAZEBO_COMPONENTS_RIGIDBODY_HH_
#define GAZEBO_COMPONENTS_RIGIDBODY_HH_

namespace gazebo
{
  namespace components
  {
    /// \brief A body that needs physics simulation
    struct RigidBody
    {
      enum Type {
        UNKNOWN = 0,
        SPHERE = 1,
        CUBE = 2,
      };

      struct SphereProperties {
        double radius;
      };

      struct CubeProperties {
        double side;
      };

      /// \brief the shape of this component
      Type type = UNKNOWN;
      /// \brief true if the component cannot move or be moved
      bool isStatic = false;
      /// \brief mass in kilograms
      double mass = 1.0;

      union {
        SphereProperties sphere;
        CubeProperties cube;
      };
    };
  }
}

#endif



