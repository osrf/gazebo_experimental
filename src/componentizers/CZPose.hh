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

#ifndef GAZEBO_COMPONENTIZERS_CZWORLDPOSE_HH__
#define GAZEBO_COMPONENTIZERS_CZWORLDPOSE_HH__

#include <unordered_map>

#include "gazebo/ecs/Componentizer.hh"
#include "gazebo/ecs/Manager.hh"

namespace gazebo
{
  namespace componentizers
  {
    /// \brief a plugin creates "Pose" component where appropriate
    class CZPose : public ecs::Componentizer
    {
      // Inherited
      public: virtual void Init();

      // Inherited
      public: virtual void FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
                  const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids);

      /// \brief map of elements to their frame names
      private: std::unordered_map<sdf::Element*, std::string> frames;
    };
  }
}

#endif
