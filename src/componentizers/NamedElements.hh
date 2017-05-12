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
#ifndef GAZEBO_COMPONENTIZERS_NAMEDELEMENTS_HH__
#define GAZEBO_COMPONENTIZERS_NAMEDELEMENTS_HH__


#include "gazebo/ecs/Componentizer.hh"
#include "gazebo/ecs/Manager.hh"

namespace gazebo
{
  namespace componentizers
  {
    /// \brief a plugin creates "Name" components for sdf elements
    class NamedElements : public ecs::Componentizer
    {
      /// \brief called whent the componentizer loads the plugin
      public: virtual void Init();

      /// \brief called when an SDF file is loaded
      /// \param[in] _mgr manager to use to create the entities and components
      /// \param[in] _element The sdf element to get data from
      public: virtual void FromSDF(ecs::Manager &_mgr, sdf::Element &_elem);
    };
  }
}

#endif
