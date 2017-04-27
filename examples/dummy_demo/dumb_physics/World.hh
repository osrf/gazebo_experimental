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

#include "dumb_physics/Body.hh"

namespace dumb_physics
{
  /// \brief Forward declaration
  class WorldPrivate;

  /// \brief A world for doing physics stuff
  class World
  {
    /// \brief Constructor
    public: World();

    /// \brief Destructor
    public: ~World();

    /// \brief Get Gravity
    /// \return Gravity vector in world coordinates.
    public: ignition::math::Vector3<double> Gravity() const;

    /// \brief Set Gravity
    /// \param[in] _gravity Gravity vector in world coordinates.
    public: void Gravity(const ignition::math::Vector3<double> &_gravity);

    /// \brief Add a body to the world
    /// \param[in] _bodyId Body id
    public: Body *AddBody(int _bodyId);

    /// \brief Get a body on the world
    /// \param[in] _bodyId Body id
    public: Body *BodyById(int _bodyId) const;

    /// \brief Remove a body from the world
    /// \param[in] _bodyId Body id
    public: void RemoveBody(int _bodyId);

    /// \brief Calculate collisions and update bodies
    /// \param[in] _dt Time step in seconds
    /// \return Set with pairs of overlapping bodies
    public: std::set<std::pair<int, int> > Update(const double _dt);

    /// \brief Set the size of the world
    /// \param[in] Size in meters
    public: void SetSize(const ignition::math::Vector3d &_size);

    /// \brief Pointer to private members.
    private: std::unique_ptr<WorldPrivate> dataPtr;
  };
}

#endif
