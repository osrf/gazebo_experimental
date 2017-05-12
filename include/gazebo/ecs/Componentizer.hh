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
#ifndef GAZEBO_ECS_COMPONENTIZER_HH__
#define GAZEBO_ECS_COMPONENTIZER_HH__

#include <unordered_map>

#include "gazebo/ecs/Entity.hh"

namespace sdf
{
  /// \brief forward declaration
  class Element;
}

namespace gazebo
{
  namespace ecs
  {
    /// \brief forward declaration
    class Manager;

    /// \brief a plugin that creates entities and components from SDF
    class Componentizer
    {
      /// \brief called when componetizer is loaded
      /// \remarks Use this method to register components
      public: virtual void Init() = 0;

      /// \brief called when an SDF file is loaded
      /// \param[in] _mgr manager to use to create the entities and components
      /// \param[in] _elem The sdf element to pull data from
      /// \param[in] _ids Maps elements to entity IDs. Makes grouping easier
      public: virtual void FromSDF(Manager &_mgr, sdf::Element &_elem,
                  const std::unordered_map<sdf::Element*, EntityId> &_ids) = 0;
    };
  }
}


#endif
