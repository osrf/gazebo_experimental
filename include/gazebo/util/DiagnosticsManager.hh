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

#ifndef GAZEBO_UTIL_DIAGNOSTICS_HH_
#define GAZEBO_UTIL_DIAGNOSTICS_HH_

#include <memory>

#include <ignition/common.hh>

namespace gazebo
{
  namespace util
  {
    /// \brief forward declaration
    class DiagnosticsManagerPrivate;

    /// \brief API for starting/stoping timers and publishing results
    class DiagnosticsManager
    {
      public: DiagnosticsManager();

      public: ~DiagnosticsManager();

      /// \brief Initialize to report diagnostics about a world.
      /// \param[in] _name describes what kind of diagnostics these are
      /// \returns true if initialization was successful
      public: bool Init(const std::string &_name);

      /// \brief called to signal the beginning of a diagnostic update
      /// \param[in] _simTime currrent simulation time
      public: void UpdateBegin(const ignition::common::Time &_simTime,
                               const ignition::common::Time &_realTime);

      /// \brief called to signal the end of a a diagnostic update
      /// \remark This publishes a diagnostics message. Any timers that have
      //          not been stopped before this call are cancelled and ignored.
      public: void UpdateEnd();

      /// \brief Start a new timer instance
      /// \param[in] _name Name of the timer.
      /// \return A pointer to the new diagnostic timer
      public: void StartTimer(const std::string &_name);

      /// \brief Stop a currently running timer.
      /// \param[in] _name Name of the timer to stop.
      public: void StopTimer(const std::string &_name);

      /// \brief private implementation
      private: std::shared_ptr<DiagnosticsManagerPrivate> dataPtr;
    };
  }
}

#endif
