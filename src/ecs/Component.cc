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


#include <gazebo/ecs/Component.hh>

using namespace gazebo::ecs;

//////////////////////////////////////////////////
Component::~Component()
{
}

//////////////////////////////////////////////////
Component::operator bool() const
{
  return this->ComponentType() != NO_COMPONENT;
}

//////////////////////////////////////////////////
NullComponent::~NullComponent()
{
}

//////////////////////////////////////////////////
const char *NullComponent::ComponentName() const
{
  return "gazebo::ecs::NullComponent";
}

//////////////////////////////////////////////////
gazebo::ecs::ComponentType NullComponent::ComponentType() const
{
  return NO_COMPONENT;
}

//////////////////////////////////////////////////
void NullComponent::ComponentType(gazebo::ecs::ComponentType _type)
{
  // Intentionally empty
}

//////////////////////////////////////////////////
void NullComponent::DeepCopy(const gazebo::ecs::Component &_other)
{
  // Intentionally empty
}

//////////////////////////////////////////////////
void NullComponent::Move(gazebo::ecs::Component &_old)
{
  // Intentionally empty
}
