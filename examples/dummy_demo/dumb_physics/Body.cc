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
#include "dumb_physics/Body.hh"


namespace dumb_physics
{
  class BodyPrivate
  {
    public: int id = -1;
    public: double mass = 1.0;
    public: double radius = 1.0;
    public: bool isStatic = false;

    public: ignition::math::Vector3<double> linearVelocity = {0, 0, 0};
    public: ignition::math::Quaternion<double> angularVelocity = {1, 0, 0, 0};
    public: ignition::math::Vector3<double> position = {0, 0, 0};
    public: ignition::math::Quaternion<double> rotation {1, 0, 0, 0};
  };
}

using namespace dumb_physics;

Body::Body()
{
  dataPtr.reset(new BodyPrivate);
}

Body::~Body()
{
}

/////////////////////////////////////////////////
int Body::Id()
{
  return this->dataPtr->id;
}

/////////////////////////////////////////////////
void Body::Id(int _id)
{
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
double Body::Mass() const
{
  return this->dataPtr->mass;
}

/////////////////////////////////////////////////
void Body::Mass(double _mass)
{
  this->dataPtr->mass = _mass;
}

/////////////////////////////////////////////////
bool Body::IsStatic() const
{
  return this->dataPtr->isStatic;
}

/////////////////////////////////////////////////
void Body::IsStatic(bool _isStatic)
{
  if (_isStatic == true)
  {
    this->dataPtr->linearVelocity.Set(0,0,0);
    this->dataPtr->angularVelocity.Set(1,0,0,0);
  }
  this->dataPtr->isStatic = _isStatic;
}

/////////////////////////////////////////////////
double Body::Radius() const
{
  return this->dataPtr->radius;
}

/////////////////////////////////////////////////
void Body::Radius(double _radius)
{
  this->dataPtr->radius = _radius;
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> Body::LinearVelocity() const
{
  return this->dataPtr->linearVelocity;
}

/////////////////////////////////////////////////
void Body::LinearVelocity(ignition::math::Vector3<double> _vel)
{
  if (!this->dataPtr->isStatic)
  {
    this->dataPtr->linearVelocity = _vel;
  }
}

/////////////////////////////////////////////////
ignition::math::Quaternion<double> Body::AngularVelocity() const
{
  return this->dataPtr->angularVelocity;
}

/////////////////////////////////////////////////
void Body::AngularVelocity(ignition::math::Quaternion<double> _vel)
{
  if (!this->dataPtr->isStatic)
  {
    this->dataPtr->angularVelocity = _vel;
  }
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> Body::Position() const
{
  return this->dataPtr->position;
}

/////////////////////////////////////////////////
void Body::Position(ignition::math::Vector3<double> _position)
{
  this->dataPtr->position = _position;
}

/////////////////////////////////////////////////
ignition::math::Quaternion<double> Body::Rotation() const
{
  return this->dataPtr->rotation;
}

/////////////////////////////////////////////////
void Body::Rotation(ignition::math::Quaternion<double> _rotation)
{
  this->dataPtr->rotation = _rotation;
}

