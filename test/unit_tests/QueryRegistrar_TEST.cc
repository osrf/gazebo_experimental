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

#include <gtest/gtest.h>
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/QueryRegistrar.hh"

// Component Types for testing
struct TC1
{
  float itemOne;
};

struct TC2
{
  float itemOne;
  int itemTwo;
};

struct TC3
{
  float itemOne;
  int itemTwo;
  double itemThree;
};

// fake declaration to avoid include
class gazebo::ecs::Manager
{
  public: std::string result;
};

/////////////////////////////////////////////////
TEST(QueryRegistrar, InitiallyNoRegistrations)
{
  gazebo::ecs::QueryRegistrar r;
  EXPECT_EQ(0, r.Registrations().size());
}

/////////////////////////////////////////////////
TEST(QueryRegistrar, RegisterOneQuery)
{
  gazebo::ecs::QueryRegistrar r;
  gazebo::ecs::EntityQuery q;
  q.AddComponent("TC1");
  gazebo::ecs::QueryCallbackPtr cb = [] (
      const gazebo::ecs::EntityQuery &_q,
      gazebo::ecs::Manager &_m, double _dt) -> void {
        _m.result = "RegisterOneQuery lambda";
      };
  r.Register(q, cb);
  ASSERT_EQ(1, r.Registrations().size());

  gazebo::ecs::Manager m;
  (r.Registrations()[0].second)(q, m, 0);
  EXPECT_EQ(std::string("RegisterOneQuery lambda"), m.result);
}

/////////////////////////////////////////////////
TEST(QueryRegistrar, QueryOrderIsPreserved)
{
  gazebo::ecs::QueryRegistrar r;
  gazebo::ecs::EntityQuery q1;
  gazebo::ecs::EntityQuery q2;
  q1.AddComponent("TC1");
  q2.AddComponent("TC2");
  gazebo::ecs::QueryCallbackPtr cb1 = [] (
      const gazebo::ecs::EntityQuery &_q,
      gazebo::ecs::Manager &_m, double _dt) -> void {
        _m.result = "first callback";
      };
  gazebo::ecs::QueryCallbackPtr cb2 = [] (
      const gazebo::ecs::EntityQuery &_q,
      gazebo::ecs::Manager &_m, double _dt) -> void {
        _m.result = "second callback";
      };
  r.Register(q1, cb1);
  r.Register(q2, cb2);
  ASSERT_EQ(2, r.Registrations().size());

  gazebo::ecs::Manager m;
  (r.Registrations()[0].second)(q1, m, 0);
  EXPECT_EQ(std::string("first callback"), m.result);
  (r.Registrations()[1].second)(q1, m, 0);
  EXPECT_EQ(std::string("second callback"), m.result);
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

