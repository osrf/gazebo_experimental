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

#include <algorithm>
#include <gtest/gtest.h>
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Entity.hh"

namespace gzecs = gazebo::ecs;

/////////////////////////////////////////////////
// Component Types for testing
struct TC1
{
  float itemOne;
};

/////////////////////////////////////////////////
struct TC2
{
  float itemOne;
  int itemTwo;
};

/////////////////////////////////////////////////
struct TC3
{
  float itemOne;
  int itemTwo;
  double itemThree;
};


/////////////////////////////////////////////////
/// \brief abuse friendship for test hooks
class gazebo::ecs::EntityComponentDatabase
{
  /// \brief create entity and return it
  public: gzecs::Entity make_entity(gzecs::EntityId id)
          {
            return gzecs::Entity(this, id);
          }
};

/////////////////////////////////////////////////
TEST(Entity, ConstructNullEntity)
{
  gzecs::Entity uut;
  EXPECT_EQ(uut, gzecs::EntityNull);
}

/////////////////////////////////////////////////
TEST(Entity, NullEntityIsNoEntity)
{
  gzecs::Entity uut;
  EXPECT_EQ(gzecs::NO_ENTITY, gzecs::EntityNull.Id());
}

/////////////////////////////////////////////////
TEST(Entity, EntityWithId)
{
  gzecs::EntityComponentDatabase testhook;
  gzecs::Entity uut = testhook.make_entity(5);
  EXPECT_EQ(5, uut.Id());
}

/////////////////////////////////////////////////
TEST(Entity, MoveConstructor)
{
  gzecs::EntityComponentDatabase testhook;
  gzecs::Entity e1 = testhook.make_entity(5);
  gzecs::Entity e2 = std::move(e1);
  EXPECT_EQ(gzecs::NO_ENTITY, e1.Id());
  EXPECT_EQ(5, e2.Id());
}

/////////////////////////////////////////////////
TEST(Entity, MoveAssignment)
{
  gzecs::EntityComponentDatabase testhook;
  gzecs::Entity e1 = testhook.make_entity(5);
  gzecs::Entity e2;
  // Expect_eq with e2 to prevent compiler optimizations using move constructor
  EXPECT_EQ(gzecs::NO_ENTITY, e2.Id());
  e2 = std::move(e1);
  EXPECT_EQ(gzecs::NO_ENTITY, e1.Id());
  EXPECT_EQ(5, e2.Id());
}

int main(int argc, char **argv)
{
  // Register types with the factory
  gazebo::ecs::ComponentFactory::Register<TC1>("TC1");
  gazebo::ecs::ComponentFactory::Register<TC2>("TC2");
  gazebo::ecs::ComponentFactory::Register<TC3>("TC3");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
