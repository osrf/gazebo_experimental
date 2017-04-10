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
    public: Body();

    public: ~Body();

    /// \brief Get id of body
    public: int Id();

    /// \brief Set id of body
    public: void Id(int _id);

    /// \brief Get mass
    public: double Mass() const;

    /// \brief Set Mass
    public: void Mass(double _mass);

    /// \returns True if the body cannot move
    public: bool IsStatic() const;

    /// \brief Set is the body is static
    public: void IsStatic(bool _isStatic);

    /// \brief Get radius of sphere
    public: double Radius() const;

    /// \brief Set radius of a sphere
    public: void Radius(double _radius);

    /// \brief Get linear velocity
    public: ignition::math::Vector3<double> LinearVelocity() const;

    /// \brief Set linear velocity
    public: void LinearVelocity(ignition::math::Vector3<double>);

    /// \brief Get angular velocity
    public: ignition::math::Quaternion<double> AngularVelocity() const;

    /// \brief Set angular velocity
    public: void AngularVelocity(ignition::math::Quaternion<double>);

    /// \brief Get Position
    public: ignition::math::Vector3<double> Position() const;

    /// \brief Set position
    public: void Position(ignition::math::Vector3<double> _position);

    /// \brief Get rotation
    public: ignition::math::Quaternion<double> Rotation() const;

    /// \brief Set rotation
    public: void Rotation(ignition::math::Quaternion<double> _rotation);

    /// \brief PIMPL
    private: std::unique_ptr<BodyPrivate> dataPtr;
  };
}

#endif
