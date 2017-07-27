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

  return NoComponentType;
}

/////////////////////////////////////////////////
std::pair<ComponentType, ComponentId>  ComponentFactory::CreateComponent(
    sdf::ElementPtr _elem)
{
  /// Make sure the component type is known.
  if (componentNameTypeMap.find(_elem->GetName()) ==
      componentNameTypeMap.end() ||
      componentStores.find(componentNameTypeMap[_elem->GetName()]) ==
      componentStores.end())
  {
    return {NoComponentType, NoComponentId};
  }

  ComponentType type = componentNameTypeMap[_elem->GetName()];

  // Create the new component
  auto componentId = componentStores[type]->Create(_elem);

  return {type, componentId};
}

/////////////////////////////////////////////////
bool ComponentFactory::RemoveComponent(
    const ComponentType _type, const ComponentId _id)
{
  if (componentStores.find(_type) == componentStores.end())
  {
    return false;
  }

  // Remove the component
  return componentStores[_type]->Remove(_id);
}

/////////////////////////////////////////////////
ComponentId ComponentFactory::CreateComponent(const ComponentType _type)
{
  auto it = componentStores.find(_type);
  if (it == componentStores.end())
    return NoComponentId;

  return it->second->Create();
}
