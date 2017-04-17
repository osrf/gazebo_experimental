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

#ifndef GAZEBO_COMPONENTS_RENDERABLE_HH_
#define GAZEBO_COMPONENTS_RENDERABLE_HH_

namespace gazebo
{
  namespace components
  {
    /// \brief A body that can be rendered
    struct Renderable
    {
      enum ShapeType {
        UNKNOWN_SHAPE = 0,
        SPHERE = 1,
        CUBE = 2,
      };

      enum MaterialType {
        UNKNOWN_MATERIAL = 0,
        COLOR = 1,
        TEXTURE = 2,
      };

      struct SphereProperties {
        double radius;
      };

      struct CubeProperties {
        double side;
      };

      struct FlatColorProperties {
        float red;
        float green;
        float blue;
      };

      struct MeshProperties {
        void *triangles;
        float scale;
      };

      /// \brief the shape of this component
      ShapeType shape = UNKNOWN_SHAPE;

      /// \brief material type of this component
      MaterialType material = UNKNOWN_MATERIAL;

      union {
        SphereProperties sphere;
        CubeProperties cube;
      };

      union {
        FlatColorProperties color;
        MeshProperties mesh;
      };
    };
  }
}

#endif
