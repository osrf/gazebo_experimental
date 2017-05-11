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

#ifndef GAZEBO_GUI_GUIDIAGNOSTICS_HH_
#define GAZEBO_GUI_GUIDIAGNOSTICS_HH_

#include <mutex>
#include <vector>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

namespace gazebo
{
  namespace gui
  {
    class GuiDiagnostics: public ignition::gui::Plugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: GuiDiagnostics();
      
      /// \brief Returns the title of the widget
      public: virtual std::string Title();

      /// \brief Destructor
      public: virtual ~GuiDiagnostics();

      /// \brief called when it is time to redraw the window
      virtual void paintEvent(QPaintEvent *);

      /// \brief callback in main thread when diagnostics come in
      public slots: void SignalDiagRx();

      /// \brief subscriber callback when new diagnostics are received
      private: void OnDiagRx(const ignition::msgs::Diagnostics &_diag);

      /// \brief returns how big this window should be
      public: virtual QSize sizeHint() const;

      /// \brief returns the minimum size of this widget
      public: virtual QSize minimumSizeHint() const;

      /// \brief holds received data that has yet to be processed
      public: std::vector<ignition::msgs::Diagnostics> msgs;

      /// \brief tools for setting up a subscriber
      private: ignition::transport::Node node;

      /// \brief mutex for accessing data
      private: std::mutex mtx;
    };
  }
}

#endif
