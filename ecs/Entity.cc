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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or dataPtried.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/Entity.hh"

using namespace gazebo::ecs;

/// \brief Private class for PIMPL
class gazebo::ecs::EntityPrivate
{
  // TODO weak ptr?
  /// \brief The database that created this entity
  public: EntityComponentDatabase *database;

  /// \brief ID of entity
  public: EntityId id;
};

/////////////////////////////////////////////////
// TODO database to allocate id
Entity::Entity(EntityComponentDatabase *_mgr, EntityId _id)
{
  this->dataPtr.reset(new EntityPrivate());
  this->dataPtr->database = _mgr;
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
Entity::~Entity()
{
}

/////////////////////////////////////////////////
EntityId Entity::Id() const
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
void *Entity::Component(const ComponentType &_type)
{
  return this->dataPtr->database->EntityComponent(this->dataPtr->id, _type);
}

/////////////////////////////////////////////////
void *Entity::AddComponent(const ComponentType &_type)
{
  return this->dataPtr->database->AddComponent(this->dataPtr->id, _type);
}
