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

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ignition/transport/Node.hh>
#include <ignition/rendering/RenderTypes.hh>

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

      /// \brief Perform the actual render updates
      private: void UpdateImpl();

      /// \brief Load render engine and create a scene
      /// \param[in] _engineName Name of rendering engine
      /// \return a camera for rendering
      private: ignition::rendering::CameraPtr LoadEngine(
          const std::string &_engineName);

      /// \brief Helper function to build a scene
      /// \param[in] _scene Pointer to the scene.
      private: void BuildScene(ignition::rendering::ScenePtr _scene);

      /// \brief Ign transport node
      private: ignition::transport::Node node;

      /// \brief Publisher to the image toipc
      private: ignition::transport::Node::Publisher pub;

      /// \brief previous update sim time
      private: ignition::common::Time prevUpdateTime;

      /// \brief previous render time
      private: ignition::common::Time prevRenderTime;

      /// \brief Sim time when update is requested.
      private: ignition::common::Time currentSimTime;

      /// \brief Rendering camera
      private: ignition::rendering::CameraPtr camera;

      /// \brief Pointer to an image to be published
      private: ignition::rendering::ImagePtr image;

      /// \brief Mutex to protect updates
      private: std::mutex mutex;

      /// \brief Thread where all rendering updates are performed in
      private: std::unique_ptr<std::thread> renderThread;

      /// \brief Flag to indicate if update is needed.
      private: bool update = false;

      /// \brief Flag to indicate if the system is about to shutdown
      private: bool stop = false;
    };
  }
}

#endif
