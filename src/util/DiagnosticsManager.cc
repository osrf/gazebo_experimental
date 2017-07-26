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

#include <map>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include "gazebo/util/DiagnosticsManager.hh"

namespace gzutil = gazebo::util;
using namespace gzutil;

class gzutil::DiagnosticsManagerPrivate
{
  /// \brief timers that are currently active
  public: std::map<std::string, ignition::common::Timer> timers;

  /// \brief update message being made for this update
  public: ignition::msgs::Diagnostics msg;

  /// \brief tools for setting up a publisher
  public: ignition::transport::Node node;

  /// \brief publisher
  public: ignition::transport::Node::Publisher pub;

  /// \brief true if initialized
  public: bool initialized = false;

  /// \brief name belonging to these diagnostics
  public: std::string name;
};

//////////////////////////////////////////////////
DiagnosticsManager::DiagnosticsManager() :
  dataPtr(new DiagnosticsManagerPrivate)
{
}

//////////////////////////////////////////////////
DiagnosticsManager::~DiagnosticsManager()
{
}

//////////////////////////////////////////////////
bool DiagnosticsManager::Init(const std::string &_name)
{
  this->dataPtr->name = _name;
  std::string topicName = "diagnostics";

  auto &pub = this->dataPtr->pub;
  auto &node = this->dataPtr->node;
  pub = node.Advertise<ignition::msgs::Diagnostics>(topicName);

  this->dataPtr->initialized = pub;
  return this->dataPtr->initialized;
}

//////////////////////////////////////////////////
void DiagnosticsManager::UpdateBegin(const ignition::common::Time &_simTime)
{
  if (this->dataPtr->initialized)
  {
    this->dataPtr->msg.mutable_sim_time()->set_sec(_simTime.sec);
    this->dataPtr->msg.mutable_sim_time()->set_nsec(_simTime.nsec);
  }
}

//////////////////////////////////////////////////
void DiagnosticsManager::UpdateEnd()
{
  if (this->dataPtr->initialized)
  {
    this->dataPtr->pub.Publish(this->dataPtr->msg);
    this->dataPtr->msg.clear_time();
    this->dataPtr->timers.clear();
  }
}

//////////////////////////////////////////////////
void DiagnosticsManager::StartTimer(const std::string &_name)
{
  if (this->dataPtr->initialized)
  {
    ignition::common::Timer timer;
    timer.Start();
    this->dataPtr->timers[_name] = timer;
  }
}

//////////////////////////////////////////////////
void DiagnosticsManager::StopTimer(const std::string &_name)
{
  if (this->dataPtr->initialized)
  {
    auto kvIter = this->dataPtr->timers.find(_name);
    if (kvIter != this->dataPtr->timers.end())
    {
      ignition::common::Time elapsed = kvIter->second.Elapsed();
      this->dataPtr->timers.erase(kvIter);

      auto diagTime = this->dataPtr->msg.add_time();
      diagTime->mutable_elapsed()->set_sec(elapsed.sec);
      diagTime->mutable_elapsed()->set_nsec(elapsed.nsec);

      // Wall time is set when the timer ends to match gazebo 8 behavior
      ignition::common::Time currentTime = ignition::common::Time::SystemTime();
      diagTime->mutable_wall()->set_sec(currentTime.sec);
      diagTime->mutable_wall()->set_nsec(currentTime.nsec);

      diagTime->set_name(this->dataPtr->name + ":" + _name);
    }
  }
}
