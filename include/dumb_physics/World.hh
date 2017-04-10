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

#ifndef _DUMB_PHYSICS_WORLD_HH_
#define _DUMB_PHYSICS_WORLD_HH_

#include <memory>
#include <set>
#include <utility>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "dumb_physics/Body.hh"

namespace dumb_physics
{
  /// \brief Forward declaration
  class WorldPrivate;

  /// \brief A world for doing physics stuff
  class World
  {
    public: World();

    public: ~World();

    /// \brief Get Gravity
    public: ignition::math::Vector3<double> Gravity();

    /// \brief Set Gravity
    public: void Gravity(ignition::math::Vector3<double> _gravity);

    /// \brief add a body to the world 
    public: Body *AddBody(int _bodyId);

    /// \brief get a body on the world 
    public: Body *GetById(int _bodyId);

    /// \brief remove a body from the world 
    public: void RemoveBody(int _bodyId);

    /// \brief do collisions and stuff
    public: std::set<std::pair<int, int> > Update(double _dt);

    /// \brief set the size of the world
    public: void SetSize(double _x, double _y, double _z);

    /// \brief PIMPL
    private: std::unique_ptr<WorldPrivate> dataPtr;
  };
}

#endif
