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
#include <set>

#include "gazebo/ecs/EntityQuery.hh"

using namespace gazebo;
using namespace ecs;

// Private data class
class gazebo::ecs::EntityQueryPrivate
{
  /// \brief list of component types that must be present on entities
  public: std::set<ComponentType> componentTypes;

  /// \brief all entities that matched the query
  public: std::set<EntityId> entityIds;
};

/////////////////////////////////////////////////
EntityQuery::EntityQuery()
: dataPtr(new EntityQueryPrivate())
{
}

/////////////////////////////////////////////////
EntityQuery::EntityQuery(EntityQuery &&_move)
: dataPtr(std::move(_move.dataPtr))
{
}

/////////////////////////////////////////////////
EntityQuery::~EntityQuery()
{
}

/////////////////////////////////////////////////
bool EntityQuery::AddComponent(const std::string &_name)
{
  return this->AddComponent(ComponentFactory::Type(_name));
}

/////////////////////////////////////////////////
bool EntityQuery::AddComponent(ComponentType _type)
{
  if (_type != NO_COMPONENT)
  {
    this->dataPtr->componentTypes.insert(_type);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
bool EntityQuery::operator==(const EntityQuery &_rhs) const
{
  return std::equal(this->dataPtr->componentTypes.begin(),
                    this->dataPtr->componentTypes.end(),
                    _rhs.dataPtr->componentTypes.begin()) &&
         std::equal(this->dataPtr->entityIds.begin(),
                    this->dataPtr->entityIds.end(),
                    _rhs.dataPtr->entityIds.begin());
}

/////////////////////////////////////////////////
bool EntityQuery::AddEntity(const EntityId _id)
{
  std::pair<std::set<EntityId>::iterator, bool> result =
    this->dataPtr->entityIds.insert(_id);

  return result.second;
}

/////////////////////////////////////////////////
void EntityQuery::RemoveEntity(const EntityId _id)
{
  this->dataPtr->entityIds.erase(_id);
}

/////////////////////////////////////////////////
void EntityQuery::Clear()
{
  this->dataPtr->entityIds.clear();
}

/////////////////////////////////////////////////
const std::set<ComponentType> &EntityQuery::ComponentTypes() const
{
  return this->dataPtr->componentTypes;
}

/////////////////////////////////////////////////
const std::set<EntityId> &EntityQuery::EntityIds() const
{
  return this->dataPtr->entityIds;
}

/////////////////////////////////////////////////
EntityQuery &EntityQuery::operator=(EntityQuery &&_rhs)
{
  this->dataPtr = std::move(_rhs.dataPtr);
}

/////////////////////////////////////////////////
bool EntityQuery::IsNull()
{
  return this->dataPtr->componentTypes.empty() &&
         this->dataPtr->entityIds.empty();
}
