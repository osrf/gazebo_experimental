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

#ifndef GAZEBO_COMPONENTS_COLLIDABLE_HH_
#define GAZEBO_COMPONENTS_COLLIDABLE_HH_

#include <gazebo/ecs/Entity.hh>

namespace gazebo
{
  namespace components
  {
    /// \brief Signals this is part of a collision group
    struct Collidable
    {
      /// \brief A grouping entity iff multiple geometry form one rigid body
      /// \description this entity is where group properties will be. These
      ///     include Inertial and WorldVelocity properties. If the group
      ///     is set to NO_ENTITY than the properties are on the same entity
      ///     as this component.
      ecs::EntityId groupId = ecs::NO_ENTITY;

      // TODO surface properties needed by physics system should go here
    };
  }
}

#endif
