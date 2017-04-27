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

#ifndef GAZEBO_PRIVATE_SYSTEMS_DUMMYRENDERING_HH_
#define GAZEBO_PRIVATE_SYSTEMS_DUMMYRENDERING_HH_

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <vector>

#include "dummy_rendering/Scene.hh"
#include "gazebo/ecs/System.hh"

namespace gazebo
{
  namespace systems
  {
    /// \brief Forward Declaration
    class EntityQueryResult;

    class DummyRendering : public ecs::System
    {
      public: virtual void Init(ecs::QueryRegistrar &_registrar);

      public: void Update(const ecs::EntityQuery &_result);

      /// \brief add a thing to be rendered
      public: void AddObjectToScene(ecs::Entity &_entity);

      /// \brief stop rendering a thing
      public: void RemoveObjectFromScene(ecs::Entity &_entity);

      /// \brief render and publish images using ignition-transport
      public: void PublishImages();

      /// \brief update internal representation with new position
      public: void UpdatePosition(ecs::Entity &_entity);

      ///  \brief Handle to rendering library
      private: dummy_rendering::Scene scene;

      /// \brief tools for setting up a publisher
      private: ignition::transport::Node node;

      /// \brief publisher
      private: ignition::transport::Node::Publisher pub;

      /// \brief accumulator that counts up current sim time
      private: ignition::common::Time nextRenderTime;
    };
  }
}
#endif
