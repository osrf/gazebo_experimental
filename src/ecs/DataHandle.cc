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

#include "gazebo/ecs/DataHandle.hh"

namespace gzecs = gazebo::ecs;
using namespace gazebo;
using namespace gzecs;


class gazebo::ecs::DataHandlePrivate
{
  /// \brief the ecs manager which created the data handle
  public: Manager *manager;

  /// \brief the ecs database which stores data
  public: EntityComponentDatabase *database;
};

//////////////////////////////////////////////////
DataHandle(Manager &_manager, EntityComponentDatabase &_database)
  : dataPtr(new DataHandlePrivate)
{
  this->dataPtr->manager = &_manager;
  this->dataPtr->database = &_database;
  this->dataPtr->database->BlockUpdate(true);
}

//////////////////////////////////////////////////
~DataHandle()
{
  this->dataPtr->database->BlockUpdate(false);
}

/////////////////////////////////////////////////
EntityId DataHandle::CreateEntity()
{
  return this->dataPtr->database->CreateEntity();
}

/////////////////////////////////////////////////
bool DataHandle::DeleteEntity(EntityId _id)
{
  return this->dataPtr->database->DeleteEntity(_id);
}

/////////////////////////////////////////////////
gazebo::ecs::Entity &DataHandle::Entity(const EntityId _id) const
{
  return this->dataPtr->database->Entity(_id);
}

/////////////////////////////////////////////////
const ignition::common::Time &DataHandle::SimulationTime() const
{
  return this->dataPtr->database->SimulationTime();
}

/////////////////////////////////////////////////
bool DataHandle::SimulationTime(const ignition::common::Time &_newTime)
{
  if (!this->dataPtr->manager->Paused())
  {
    this->dataPtr->database->SimulationTime(nextSimTime);
    return true;
  }
  return false;
}
