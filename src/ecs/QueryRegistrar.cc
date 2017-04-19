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

#include <utility>
#include <vector>

#include "gazebo/ecs/QueryRegistrar.hh"

using namespace gazebo::ecs;

class gazebo::ecs::QueryRegistrarPrivate
{
  /// \brief queries and callbacks that have been registered
  public: std::vector<QueryRegistration> queryCallbacks;
};

/////////////////////////////////////////////////
QueryRegistrar::QueryRegistrar() :
  dataPtr(new QueryRegistrarPrivate)
{
}

/////////////////////////////////////////////////
QueryRegistrar::~QueryRegistrar()
{
}

/////////////////////////////////////////////////
void QueryRegistrar::Register(const EntityQuery &_q, QueryCallbackPtr _cb)
{
  QueryRegistration r = {_q, _cb};
  this->dataPtr->queryCallbacks.push_back(r);
}

/////////////////////////////////////////////////
std::vector<QueryRegistration> QueryRegistrar::Registrations() const
{
  return this->dataPtr->queryCallbacks;
}

