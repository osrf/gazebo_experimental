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

#ifndef GAZEBO_SYSTEMS_RENDERING_RENDERSYSTEM_HH_
#define GAZEBO_SYSTEMS_RENDERING_RENDERSYSTEM_HH_

#include <ignition/transport.hh>
#include <ignition/rendering.hh>

#include "gazebo/ecs/System.hh"

namespace gazebo
{
  namespace systems
  {
    class RenderSystem : public ecs::System
    {
      /// \brief Constructor
      public: RenderSystem();

      /// \brief Destructor
      public: ~RenderSystem();

      // Documentation Inherited
      public: virtual void Init(ecs::QueryRegistrar &_registrar);

      /// \brief Update the rendering system
      /// \param[in] _result EntityQuery with results fromm a registered query
      public: void Update(const ecs::EntityQuery &_result);

      /// \brief Create a camera for rendering
      /// \param[in] _engineName Name of rendering engine
      private: ignition::rendering::CameraPtr CreateCamera(
          const std::string &_engineName);

      /// \brief Helper function to build a scene
      /// \param[in] _scene Pointer to the scene.
      private: void BuildScene(ignition::rendering::ScenePtr _scene);

      /// \brief Ign transport node
      public: ignition::transport::Node node;

      /// \brief Publisher to the image toipc
      public: ignition::transport::Node::Publisher pub;

      /// \brief previous update sim time
      public: ignition::common::Time prevUpdateTime;

      /// \brief previous render time
      public: ignition::common::Time prevRenderTime;

      /// \brief Rendering camera
      public: ignition::rendering::CameraPtr camera;

      /// \brief Pointer to an image to be published
      public: ignition::rendering::ImagePtr image;
    };
  }
}

#endif
