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

#ifndef GAZEBO_GUI_GUIRENDERWIDGET_HH_
#define GAZEBO_GUI_GUIRENDERWIDGET_HH_

#include <ignition/gui/qt.h>
#include <ignition/gui/Plugin.hh>

#include <ignition/rendering/RenderTypes.hh>

namespace gazebo
{
  namespace gui
  {
    class GuiRenderWidget: public ignition::gui::Plugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: GuiRenderWidget();

      /// \brief Destructor
      public: virtual ~GuiRenderWidget();

      /// \brief Qt paint event.
      protected: virtual void paintEvent(QPaintEvent *_e);

      protected: virtual void showEvent(QShowEvent *_e);

      protected: virtual void resizeEvent(QResizeEvent *_e);

      protected: virtual void moveEvent(QMoveEvent *_e);


      /// \brief Override paintEngine to stop Qt From trying to draw on top of
      /// render window.
      /// \return NULL.
      protected: virtual QPaintEngine *paintEngine() const;

      private: void CreateRenderWindow();

      private: QTimer *updateTimer = nullptr;

      private: ignition::rendering::RenderWindowPtr renderWindow;
      private: ignition::rendering::CameraPtr camera;
    };
  }
}

#endif
