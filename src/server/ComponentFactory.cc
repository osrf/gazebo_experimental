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
#include "gazebo/server/ComponentFactory.hh"

using namespace gazebo::server;

std::mutex ComponentFactory::mutex;

std::map<std::string, ComponentType> ComponentFactory::componentNameTypeMap;

std::map<ComponentType, std::unique_ptr<ComponentStoreBase>>
ComponentFactory::componentStores;

std::map<EntityId, std::vector<std::pair<ComponentType, ComponentId>>>
ComponentFactory::entityComponentMap;

ComponentType ComponentStoreBase::typeCounter = 0;

/////////////////////////////////////////////////
ComponentStoreBase::ComponentStoreBase() : type(typeCounter++)
{
}

/////////////////////////////////////////////////
ComponentType ComponentStoreBase::Type() const
{
  return this->type;
}

/////////////////////////////////////////////////
ComponentType ComponentFactory::Type(const std::string &_name)
{
  std::lock_guard<std::mutex> lock(mutex);

  if (componentNameTypeMap.find(_name) != componentNameTypeMap.end())
    return componentNameTypeMap[_name];

  return NO_COMPONENT;
}

/////////////////////////////////////////////////
bool ComponentFactory::CreateComponent(EntityId _id, sdf::ElementPtr _elem)
{
  /// Make sure the component type is known.
  if (componentNameTypeMap.find(_elem->GetName()) ==
      componentNameTypeMap.end() ||
      componentStores.find(componentNameTypeMap[_elem->GetName()]) ==
      componentStores.end())
  {
    return false;
  }

  ComponentType type = componentNameTypeMap[_elem->GetName()];

  // Create the new component
  auto componentId = componentStores[type]->Create(_elem);

  // Attach the component to the entity
  entityComponentMap[_id].push_back(std::make_pair(type, componentId));

  return true;
}

/////////////////////////////////////////////////
bool ComponentFactory::RemoveComponents(EntityId _id)
{
  bool result = true;

  for (auto &ec : entityComponentMap[_id])
  {
    result = result && RemoveComponent(ec);
  }

  return result;
}

/////////////////////////////////////////////////
bool ComponentFactory::RemoveComponent(
    const std::pair<ComponentType, ComponentId> &_key)
{
  if (componentStores.find(_key.first) == componentStores.end())
  {
    return -1;
  }

  // Remove the component
  return componentStores[_key.first]->Remove(_key.second);
}

/////////////////////////////////////////////////
bool ComponentFactory::EntityMatches(EntityId _id,
    const std::set<ComponentType> &_types)
{
  auto it = entityComponentMap.find(_id);

  if (it != entityComponentMap.end())
  {
    for (auto const &componentType : _types)
    {
      bool hasComponent = false;
      for (auto const &componentKey : it->second)
      {
        auto type = componentStores[componentKey.first]->Type();
        if (componentType == type)
        {
          hasComponent = true;
          break;
        }
      }

      if (!hasComponent)
        return false;
    }
  }
  else
  {
    return false;
  }

  return true;
}
