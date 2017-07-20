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

#include "gazebo/ecs/System.hh"

using namespace gazebo;
using namespace ecs;

/// \brief Private data class for a System
class gazebo::ecs::SystemPrivate
{
  /// \brief Pointer the the ECS manager
  public: ecs::Manager *mgr;
};

/////////////////////////////////////////////////
System::System()
: dataPtr(new SystemPrivate())
{
}

/////////////////////////////////////////////////
ecs::Manager &System::Manager()
{
  return *(this->dataPtr->mgr);
}

/////////////////////////////////////////////////
void System::Manager(ecs::Manager *_mgr)
{
  this->dataPtr->mgr = _mgr;
}

/////////////////////////////////////////////////
System::~System()
{
  // Pure virtual destructors still have to be defined
}
