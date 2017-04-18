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

#ifndef _DUMB_PHYSICS_BODY_HH_
#define _DUMB_PHYSICS_BODY_HH_

#include <memory>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

namespace dumb_physics
{
  /// \brief Forward declaration
  class BodyPrivate;

  /// \brief a rigid body in the world (Only supports spheres)
  class Body
  {
    /// \brief Constructor
    public: Body();

    /// \brief Destructor
    public: ~Body();

    /// \brief Get id of body
    /// \return Body id
    public: int Id() const;

    /// \brief Set id of body
    /// \param[in] Body id
    public: void Id(const int _id);

    /// \brief Get mass
    /// \return Body mass in kg
    public: double Mass() const;

    /// \brief Set Mass
    /// \param[in] Body mass in kg
    public: void Mass(const double _mass);

    /// \brief Get whether this body is static
    /// \returns True if the body cannot move
    public: bool IsStatic() const;

    /// \brief Set is the body is static
    /// \param[in] True if body can't move.
    public: void IsStatic(const bool _isStatic);

    /// \brief Get radius of sphere
    /// \return Radius in meters
    public: double Radius() const;

    /// \brief Set radius of a sphere
    /// \param[in] Radius in meters
    public: void Radius(const double _radius);

    /// \brief Get linear velocity
    /// \return Linear velocity in m/s
    public: ignition::math::Vector3<double> LinearVelocity() const;

    /// \brief Set linear velocity
    /// \param[in] _linVel Linear velocity in m/s
    public: void LinearVelocity(const ignition::math::Vector3<double> &_linVel);

    /// \brief Get angular velocity
    /// \return Angular velocity in rad/s
    public: ignition::math::Quaternion<double> AngularVelocity() const;

    /// \brief Set angular velocity
    /// \param[in] _angVel Angular velocity in rad/s
    public: void AngularVelocity(const ignition::math::Quaternion<double> &_angVel);

    /// \brief Get Position
    /// \return Position in m
    public: ignition::math::Vector3<double> Position() const;

    /// \brief Set position
    /// \param[in] _position Position in meters
    public: void Position(const ignition::math::Vector3<double> &_position);

    /// \brief Get rotation
    /// \return Rotation in rad
    public: ignition::math::Quaternion<double> Rotation() const;

    /// \brief Set rotation
    /// \param[in] _rotation Rotation in rad
    public: void Rotation(const ignition::math::Quaternion<double> &_rotation);

    /// \brief Pointer to private data.
    private: std::unique_ptr<BodyPrivate> dataPtr;
  };
}

#endif
