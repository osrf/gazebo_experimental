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

#ifndef GAZEBO_SYSTEMS_TIMESYSTEM_HH_
#define GAZEBO_SYSTEMS_TIMESYSTEM_HH_

#include <memory>

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/System.hh"

namespace gazebo
{
  namespace systems
  {
    class TimeSystemPrivate;

    /// \brief ECSystem to do physics
    class TimeSystem : public ecs::System
    {
      /// \brief Cosntructor
      public: TimeSystem();

      /// \brief Called when the system is loaded
      /// \param[in] Registrar used to register callbacks for query results
      public: virtual void Init(ecs::QueryRegistrar &_registrar);

      /// \brief Called every physics update with global configuration
      /// \param[in] _result EntityQuery with results fromm a registered query
      protected: void UpdateConfig(const ecs::EntityQuery &_result);

      /// \brief Callback for world control service, which takes care of play, pause,
      /// step, etc.
      /// \param[in] _req World control request
      /// \param[out] _rep Empty reply (for now, we may want to return something)
      /// \param[out] _result True if request was successfully completed.
      private: void WorldControlService(const ignition::msgs::WorldControl &_req,
                   ignition::msgs::Empty &/*_rep*/, bool &_result);

      /// \internal
      /// \brief Pointer to private data
      private: std::unique_ptr<TimeSystemPrivate> dataPtr;
    };
  }
}
#endif

