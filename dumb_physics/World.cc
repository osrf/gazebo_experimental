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

#include <cmath>
#include <map>

#include "dumb_physics/World.hh"

namespace dumb_physics
{
  class WorldPrivate
  {
    /// \brief Gravity vector in m/s^2
    public: ignition::math::Vector3d gravity = {0, 0, 0};

    /// \brief Number of iterations elapsed
    public: unsigned int iterations = 1;

    /// \brief Map of bodies in the world
    public: std::map<int, std::shared_ptr<Body> > bodies;

    /// \brief Size of the world in meters
    public: ignition::math::Vector3d size = {2.0, 2.0, 2.0};
  };
}

using namespace dumb_physics;

/////////////////////////////////////////////////
World::World()
{
  dataPtr.reset(new WorldPrivate);
}

/////////////////////////////////////////////////
World::~World()
{
}

/////////////////////////////////////////////////
ignition::math::Vector3d World::Gravity() const
{
  return this->dataPtr->gravity;
}

/////////////////////////////////////////////////
void World::Gravity(const ignition::math::Vector3d &_gravity)
{
  this->dataPtr->gravity = _gravity;
}

/////////////////////////////////////////////////
Body *World::AddBody(int _bodyId)
{
  auto bodyIter = this->dataPtr->bodies.find(_bodyId);
  if (bodyIter == this->dataPtr->bodies.end())
  {
    Body *body = new Body;
    body->Id(_bodyId);
    this->dataPtr->bodies[_bodyId] = std::move(std::shared_ptr<Body>(body));
  }
  return this->dataPtr->bodies[_bodyId].get();
}

/////////////////////////////////////////////////
Body *World::BodyById(int _bodyId) const
{
  Body *body = nullptr;
  auto bodyIter = this->dataPtr->bodies.find(_bodyId);
  if (bodyIter != this->dataPtr->bodies.end())
    body = bodyIter->second.get();
  return body;
}

/////////////////////////////////////////////////
void World::RemoveBody(int _bodyId)
{
  this->dataPtr->bodies.erase(_bodyId);
}

/////////////////////////////////////////////////
void World::SetSize(const ignition::math::Vector3d &_size)
{
  this->dataPtr->size = _size;
}

/////////////////////////////////////////////////
std::set<std::pair<int, int> > World::Update(const double _dt)
{
  // loop through all bodies and advance position by velocity
  for (auto kv : this->dataPtr->bodies)
  {
    std::shared_ptr<Body> &body = kv.second;

    // Add in gravity too
    body->LinearVelocity(
        body->LinearVelocity() + (this->dataPtr->gravity * _dt));

    body->Position(body->Position() + (body->LinearVelocity() * _dt));
    body->Rotation(body->Rotation() + (body->AngularVelocity() * _dt));
  }

  std::set<std::pair<int, int> > overlappingBodies;

  // Collision detection between spheres
  for (auto kv : this->dataPtr->bodies)
  {
    std::shared_ptr<Body> &body = kv.second;
    for (auto kv2 : this->dataPtr->bodies)
    {
      std::shared_ptr<Body> &other = kv2.second;
      if (body->Id() == other->Id())
      {
        // same body
        continue;
      }
      if (body->Position().Distance(other->Position()) <
          body->Radius() + other->Radius())
      {
        // They overlap
        int firstId = body->Id();
        int secondId = other->Id();
        if (firstId > secondId)
        {
          firstId = other->Id();
          secondId = body->Id();
        }
        overlappingBodies.insert(std::make_pair(firstId, secondId));
      }
    }
  }

  // Collision response between spheres
  // https://studiofreya.com/3d-math-and-physics/
  // simple-sphere-sphere-collision-detection-and-collision-response/
  for (auto overlap : overlappingBodies)
  {
    Body *body1 = this->BodyById(overlap.first);
    Body *body2 = this->BodyById(overlap.second);
    ignition::math::Vector3<double> basis;
    basis = body1->Position() - body2->Position();
    basis.Normalize();

    ignition::math::Vector3<double> basis1x =
      basis * basis.Dot(body1->LinearVelocity());
    ignition::math::Vector3<double> basis1y =
      body1->LinearVelocity() - basis1x;

    ignition::math::Vector3<double> basis2x =
      basis * basis.Dot(body2->LinearVelocity());
    ignition::math::Vector3<double> basis2y =
      body2->LinearVelocity() - basis2x;

    double m1 = body1->Mass();
    double m2 = body2->Mass();

    body1->LinearVelocity(basis1x * ((m1 - m2) / (m1 + m2))
        + basis2x * ((2 * m2) / (m1 + m2)) + basis1y);
    body2->LinearVelocity(basis1x * ((2 * m1) / (m1 + m2))
        + basis2x * ((m2 - m1) / (m1 + m2)) + basis2y);
  }

  // Simple but incorrect collision detection and response at bounds
  for (auto kv : this->dataPtr->bodies)
  {
    std::shared_ptr<Body> &body = kv.second;
    ignition::math::Vector3<double> pose = body->Position();
    ignition::math::Vector3<double> vel = body->LinearVelocity();
    double x2 = this->dataPtr->size.X() / 2.0;
    double y2 = this->dataPtr->size.Y() / 2.0;
    double z2 = this->dataPtr->size.Z() / 2.0;
    bool collision = false;

    // X
    if (pose.X() - body->Radius() < -x2)
    {
      vel.X(std::abs(vel.X()));
      pose.X(-x2 + body->Radius());
    }
    else if (pose.X() + body->Radius() > x2)
    {
      vel.X(std::abs(vel.X()) * -1.0);
      pose.X(x2 - body->Radius());
    }
    // Y
    if (pose.Y() - body->Radius() < -y2)
    {
      vel.Y(std::abs(vel.Y()));
      pose.Y(-y2 + body->Radius());
    }
    else if (pose.Y() + body->Radius() > y2)
    {
      vel.Y(std::abs(vel.Y()) * -1.0);
      pose.Y(y2 - body->Radius());
    }
    // Z
    if (pose.Z() - body->Radius() < -z2)
    {
      vel.Z(std::abs(vel.Z()));
      pose.Z(-z2 + body->Radius());
    }
    else if (pose.Z() + body->Radius() > z2)
    {
      vel.Z(std::abs(vel.Z()) * -1.0);
      pose.Z(z2 - body->Radius());
    }

    body->LinearVelocity(vel);
    body->Position(pose);
  }

  return overlappingBodies;
}


