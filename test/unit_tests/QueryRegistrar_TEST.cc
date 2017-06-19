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
#include "util/TestComponent.hh"
#include "gazebo/ecs/QueryRegistrar.hh"

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
  q.AddComponent<TestComponent1>();
  std::string sentinel;
  gazebo::ecs::QueryCallback cb = [&sentinel] (
      const gazebo::ecs::EntityQuery &_q) -> void {
        sentinel = "RegisterOneQuery lambda";
      };
  r.Register(q, cb);
  ASSERT_EQ(1, r.Registrations().size());

  (r.Registrations()[0].second)(q);
  EXPECT_EQ(std::string("RegisterOneQuery lambda"), sentinel);
}

/////////////////////////////////////////////////
TEST(QueryRegistrar, QueryOrderIsPreserved)
{
  gazebo::ecs::QueryRegistrar r;
  gazebo::ecs::EntityQuery q1;
  gazebo::ecs::EntityQuery q2;
  q1.AddComponent<TestComponent1>();
  q2.AddComponent<TestComponent1>();
  std::string sentinel;
  gazebo::ecs::QueryCallback cb1 = [&sentinel] (
      const gazebo::ecs::EntityQuery &_q) -> void {
        sentinel = "first callback";
      };
  gazebo::ecs::QueryCallback cb2 = [&sentinel] (
      const gazebo::ecs::EntityQuery &_q) -> void {
        sentinel = "second callback";
      };
  r.Register(q1, cb1);
  r.Register(q2, cb2);
  ASSERT_EQ(2, r.Registrations().size());

  (r.Registrations()[0].second)(q1);
  EXPECT_EQ(std::string("first callback"), sentinel);
  (r.Registrations()[1].second)(q2);
  EXPECT_EQ(std::string("second callback"), sentinel);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
