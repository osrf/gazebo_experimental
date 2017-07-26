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

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>

#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "gazebo/components/TimeInfo.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/EntityQuery.hh"
#include "TimeSystem.hh"

class gazebo::systems::TimeSystemPrivate
{
  /// \brief tools for setting up a publisher
  public: ignition::transport::Node node;

  /// \brief publisher
  public: ignition::transport::Node::Publisher pub;
};

namespace gzsys = gazebo::systems;
using namespace gzsys;
using namespace gazebo;

/////////////////////////////////////////////////
TimeSystem::TimeSystem() : dataPtr(new TimeSystemPrivate)
{
}

/////////////////////////////////////////////////
void TimeSystem::Init(ecs::QueryRegistrar &_registrar)
{
  // Query for time info
  ecs::EntityQuery configQuery;
  if (!configQuery.AddComponent("gazebo::components::TimeInfo"))
  {
    ignerr << "Undefined component[gazebo::components::TimeInfo]\n";
  }
  else
  {
    _registrar.Register(configQuery,
        std::bind(&TimeSystem::UpdateConfig, this, std::placeholders::_1));
  }

  // World stats publisher
  this->dataPtr->pub =
      this->dataPtr->node.Advertise<ignition::msgs::WorldStatistics>(
      "/world_stats");

  // World control service
  if (!this->dataPtr->node.Advertise("/world_control",
      &TimeSystem::WorldControlService, this))
  {
    ignerr << "Error advertising world control service." << std::endl;
  }
}

/////////////////////////////////////////////////
void TimeSystem::UpdateConfig(const ecs::EntityQuery &_result)
{
  ecs::Manager &mgr = this->Manager();
  auto const &entityIds = _result.EntityIds();
  if (!entityIds.empty())
  {
    // only consider the first entity
    auto &entity = mgr.Entity(*entityIds.begin());
    auto difference = entity.IsDifferent<components::TimeInfo>();

    if (difference == ecs::WAS_CREATED || difference == ecs::WAS_MODIFIED)
    {
      auto const *time = entity.Component<components::TimeInfo>();

      ignition::msgs::WorldStatistics msg;
      ignition::msgs::Time timeMsg;

      timeMsg.set_sec(time->simTime.sec);
      timeMsg.set_nsec(time->simTime.nsec);
      msg.mutable_sim_time()->CopyFrom(timeMsg);

      timeMsg.set_sec(time->realTime.sec);
      timeMsg.set_nsec(time->realTime.nsec);
      msg.mutable_real_time()->CopyFrom(timeMsg);

      msg.set_paused(time->paused);

      this->dataPtr->pub.Publish(msg);
    }
  }
}

//////////////////////////////////////////////////
void TimeSystem::WorldControlService(const ignition::msgs::WorldControl &_req,
    ignition::msgs::Empty &/*_rep*/, bool &_result)
{
  _result = false;
  ecs::Manager &mgr = this->Manager();

  if (_req.has_pause())
  {
    if (_req.pause())
      mgr.BeginPause();
    else
      mgr.EndPause();

    _result = true;
  }
}

/////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::TimeSystem,
                                  gazebo::ecs::System)
