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

#include "gazebo/server/Entity.hh"

using namespace gazebo::server;

/// \brief Private class for PIMPL
class gazebo::server::EntityPrivate
{
  /// \brief ID of entity
  public: EntityId id = NoEntity;
};

/////////////////////////////////////////////////
Entity::Entity()
: dataPtr(new EntityPrivate())
{
}

/////////////////////////////////////////////////
// TODO database to allocate id
Entity::Entity(EntityId _id)
: dataPtr(new EntityPrivate())
{
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
Entity::Entity(Entity &&_entity)
: dataPtr(std::move(_entity.dataPtr))
{
  _entity.dataPtr.reset(new EntityPrivate());
}

/////////////////////////////////////////////////
Entity::~Entity()
{
}

/////////////////////////////////////////////////
Entity &Entity::operator=(Entity &&_entity)
{
  this->dataPtr = std::move(_entity.dataPtr);
  _entity.dataPtr.reset(new EntityPrivate());
  return *this;
}

/////////////////////////////////////////////////
bool Entity::operator==(const Entity &_entity) const
{
  return this->dataPtr->id == _entity.dataPtr->id;
}

/////////////////////////////////////////////////
EntityId Entity::Id() const
{
  return this->dataPtr->id;
}
