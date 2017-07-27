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

#include <new>
#include <ignition/math.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief A geometry
    struct Geometry
    {
      /// \brief Sphere properties
      struct SphereProperties
      {
        /// \brief Radius in meters
        double radius;
      };

      /// \brief Rectangular box properties
      struct BoxProperties
      {
        /// \brief Side length in meters
        ignition::math::Vector3d size;
      };

      /// \brief Cylinder properties
      struct CylinderProperties
      {
        /// \brief radius in meters
        double radius;
        /// \brief height in meters
        double length;
      };

      /// \brief Possible geometry types
      enum Type
      {
        UNKNOWN = 0,
        SPHERE = 1,
        BOX = 2,
        CYLINDER = 3,
      };

      /// \brief constructor
      Geometry() : type(UNKNOWN)
      {
      }

      /// \brief copy constructor
      Geometry(const Geometry &_other)
      {
        // Unions make things hard :(
        // Use placement new to invoke the right copy constructor
        switch (_other.type)
        {
          case SPHERE:
            new (&sphere) SphereProperties(_other.sphere);
          case BOX:
            new (&box) BoxProperties(_other.box);
          case CYLINDER:
            new (&cylinder) CylinderProperties(_other.cylinder);
          default:
            break;
        }
      }

      /// \brief destructor
      ~Geometry()
      {
        switch (this->type)
        {
          case SPHERE:
            sphere.~SphereProperties();
          case BOX:
            box.~BoxProperties();
          case CYLINDER:
            cylinder.~CylinderProperties();
          default:
            break;
        }
      }

      /// \brief the shape of this component
      Type type;

      /// \brief Only one set of geometry properties can be defined at a time.
      union
      {
        SphereProperties sphere;
        BoxProperties box;
        CylinderProperties cylinder;
      };
    };
  }
}

#endif

