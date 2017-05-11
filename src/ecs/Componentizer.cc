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
#include <vector>

#include "gazebo/ecs/Componentizer.hh"
#include "gazebo/ecs/ComponentizerPlugin.hh"


namespace gzecs = gazebo::ecs;
using namespace gzecs;

/// \brief Private implementation
class gzecs::ComponentizerPrivate
{
  /// \brief plugins that can create components
  public: std::vector<std::unique_ptr<ComponentizerPlugin> > plugins;
};

Componentizer::Componentizer(Manager &_mgr) :
  dataPtr(new ComponentizerPrivate)
{
}

Componentizer::~Componentizer()
{
}

void Componentizer::AddPlugin(std::unique_ptr<ComponentizerPlugin> _plugin)
{
  this->dataPtr->plugins.push_back(std::move(_plugin));
}

void Componentizer::FromSDF(const sdf::SDF &_sdf)
{
  for (auto &plugin : this->dataPtr->plugins)
  {
    // Give them the SDF
    // TODO
  }
}
