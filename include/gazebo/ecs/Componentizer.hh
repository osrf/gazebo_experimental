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
#ifndef GAZEBO_ECS_COMPONENTIZER_HH_
#define GAZEBO_ECS_COMPONENTIZER_HH_

#include <memory>
#include <sdf/sdf.hh>

namespace gazebo
{
  namespace ecs
  {
    /// \brief forward declaration
    class Manager;

    /// \brief forward declaration
    class ComponentizerPlugin;

    /// \brief forward declaration
    class ComponentizerPrivate;

    class Componentizer
    {
      /// \brief constructor
      public: Componentizer();

      /// \brief destructor
      public: ~Componentizer();

      /// \brief Add a componentizer plugin
      /// \param[in] _plugin a loaded plugin
      public: void AddPlugin(std::unique_ptr<ComponentizerPlugin> _plugin);

      /// \brief turn SDF into entities and components
      /// \param[in] _mgr Manager used to create entities and components
      /// \param[in] _sdf a parsed sdformat file
      public: void FromSDF(Manager &_mgr, const sdf::SDF &_sdf);

      /// \brief private implementation
      private: std::shared_ptr<ComponentizerPrivate> dataPtr;
    };
  }
}

#endif
