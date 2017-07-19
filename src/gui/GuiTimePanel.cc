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

#include <algorithm>
#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <ignition/common/Time.hh>
#include <map>

#include "GuiTimePanel.hh"

namespace gazebo
{
namespace gui
{
  class GuiTimePanelPrivate
  {
    public: QLabel *simTime;
    public: QLabel *realTime;
    public: ignition::msgs::Diagnostics msg;

    /// \brief Mutex to protect msg
    public: std::mutex mutex;

    /// \brief tools for setting up a subscriber
    public: ignition::transport::Node node;
  };
}
}

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
GuiTimePanel::GuiTimePanel()
  : Plugin(), dataPtr(new GuiTimePanelPrivate)
{
  this->title = "Time panel";

  this->dataPtr->simTime = new QLabel("N/A");
  this->dataPtr->realTime = new QLabel("N/A");

  auto mainLayout = new QGridLayout();
  mainLayout->addWidget(new QLabel("Sim time"), 0, 0);
  mainLayout->addWidget(this->dataPtr->simTime, 0, 1);

  mainLayout->addWidget(new QLabel("Real time"), 1, 0);
  mainLayout->addWidget(this->dataPtr->realTime, 1, 1);

  mainLayout->setSizeConstraint(QLayout::SetFixedSize);
  this->setLayout(mainLayout);

  // Subscribe to get images
  std::string topic = "diagnostics";
  if (!this->dataPtr->node.Subscribe(topic, &GuiTimePanel::OnDiagnosticsMsg,
      this))
  {
    ignwarn << "Unable to subscribe to diagnostics" << std::endl;
  }
}

/////////////////////////////////////////////////
GuiTimePanel::~GuiTimePanel()
{
}

/////////////////////////////////////////////////
void GuiTimePanel::ProcessMsg()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  ignition::common::Time time;
  time.sec = this->dataPtr->msg.sim_time().sec();
  time.nsec = this->dataPtr->msg.sim_time().nsec();

  this->dataPtr->simTime->setText(QString::fromStdString(
        time.FormattedString()));

  time.sec = this->dataPtr->msg.real_time().sec();
  time.nsec = this->dataPtr->msg.real_time().nsec();

  this->dataPtr->realTime->setText(QString::fromStdString(
        time.FormattedString()));
}

/////////////////////////////////////////////////
void GuiTimePanel::OnDiagnosticsMsg(const ignition::msgs::Diagnostics &_msg)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->msg.CopyFrom(_msg);

  // Signal to GUI (main) thread that diagnostics are in
  QMetaObject::invokeMethod(this, "ProcessMsg");
}

// Register this plugin
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::gui::GuiTimePanel,
                                  ignition::gui::Plugin);
