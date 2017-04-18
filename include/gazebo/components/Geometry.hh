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

#ifndef GAZEBO_COMPONENTS_GEOMETRY_HH_
#define GAZEBO_COMPONENTS_GEOMETRY_HH_

namespace gazebo
{
  namespace components
  {
    /// \brief A geometry
    struct Geometry
    {
      /// \brief Possible geometry types
      enum Type
      {
        /// \brief Unknown geometry
        UNKNOWN = 0,
        /// \brief Sphere geometry
        SPHERE = 1,
        /// \brief Cube geometry
        CUBE = 2,
      };

      /// \brief the shape of this component
      Type type = UNKNOWN;

      /// \brief Sphere properties
      struct SphereProperties
      {
        /// \brief Radius in meters
        double radius;
      };

      /// \brief Cube properties
      struct CubeProperties
      {
        /// \brief Side length in meters
        double side;
      };

      /// \brief Only one set of geometry properties can be defined at a time.
      union
      {
        /// \brief Sphere geometries will have sphere properties
        SphereProperties sphere;
        CubeProperties cube;
      };
    };
  }
}

#endif

