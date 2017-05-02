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

#ifndef GAZEBO_GUI_GUIDISPLAYIMAGE_HH_
#define GAZEBO_GUI_GUIDISPLAYIMAGE_HH_

#include <mutex>

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>
#include <ignition/transport.hh>
#include <ignition/msgs.hh>

namespace gazebo
{
  namespace gui
  {
    class GuiDisplayImage: public ignition::gui::Plugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: GuiDisplayImage();
      
      /// \brief Returns the title of the widget
      public: virtual std::string Title();

      /// \brief Destructor
      public: virtual ~GuiDisplayImage();

      /// \brief Update from rx'd RGB_INT8
      public: void UpdateFromRgbInt8();

      /// \brief callback in main thread when image changes
      public slots: void OnImageChanged();

      /// \brief subscriber callback when new image is received
      public: void OnImageReceived(const ignition::msgs::Image &_img);

      /// \brief holds the image data to display
      public: QPixmap pixmap;

      /// \brief holds data to set as the next image
      public: ignition::msgs::Image img;

      /// \brief pointer label in layout;
      public: QLabel *label;

      /// \brief tools for setting up a subscriber
      private: ignition::transport::Node node;

      /// \brief mutex for accessing image data
      private: std::mutex mtx;
    };
  }
}

#endif
