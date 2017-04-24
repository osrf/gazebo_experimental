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
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"

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
class TestHookSystem : public gzecs::System
{
  /// \brief String that's set on update
  public: std::string sentinel;

  public: double lastDelta = 0;

  public: virtual gzecs::EntityQuery Init()
    {
      this->sentinel = "Init Ran";
      return gzecs::EntityQuery();
    }

  public: virtual void Update(double _dt, const gzecs::EntityQuery &_result,
              gzecs::Manager &_mgr)
    {
      this->sentinel = "Update Ran";
      this->lastDelta = _dt;
    }
};

/////////////////////////////////////////////////
TEST(Manager, CreateEntity)
{
  gzecs::Manager mgr;
  gzecs::EntityId id = mgr.CreateEntity();
  EXPECT_NE(gzecs::NO_ENTITY, id);
}

/////////////////////////////////////////////////
TEST(Manager, GetEntity)
{
  gzecs::Manager mgr;
  gzecs::EntityId id = mgr.CreateEntity();
  gzecs::Entity &entity = mgr.Entity(id);
  EXPECT_EQ(id, entity.Id());
}

/////////////////////////////////////////////////
TEST(Manager, DeleteEntity)
{
  gzecs::Manager mgr;
  gzecs::EntityId id = mgr.CreateEntity();
  mgr.UpdateSystems(0);
  mgr.DeleteEntity(id);
  mgr.UpdateSystems(0);
  EXPECT_EQ(gzecs::NO_ENTITY, mgr.Entity(id).Id());
}

/////////////////////////////////////////////////
TEST(Manager, LoadSystem)
{
  gzecs::Manager mgr;
  TestHookSystem *raw = new TestHookSystem;
  std::unique_ptr<gzecs::System> sys(dynamic_cast<gzecs::System*>(raw));
  mgr.LoadSystem(std::move(sys));
  EXPECT_EQ(std::string("Init Ran"), raw->sentinel);
  mgr.UpdateSystems(1.09);
  EXPECT_EQ(std::string("Update Ran"), raw->sentinel);
  EXPECT_FLOAT_EQ(1.09, raw->lastDelta);
}

/////////////////////////////////////////////////
TEST(Manager, PauseCount)
{
  gzecs::Manager mgr;
  EXPECT_EQ(1, mgr.BeginPause());
  EXPECT_EQ(2, mgr.BeginPause());
  EXPECT_EQ(3, mgr.BeginPause());
  EXPECT_EQ(2, mgr.EndPause());
  EXPECT_EQ(1, mgr.EndPause());
  EXPECT_EQ(0, mgr.EndPause());
}

/////////////////////////////////////////////////
TEST(Manager, InitiallyNotPaused)
{
  gzecs::Manager mgr;
  EXPECT_FALSE(mgr.Paused());
}

/////////////////////////////////////////////////
TEST(Manager, PauseUnpause)
{
  gzecs::Manager mgr;
  mgr.BeginPause();
  EXPECT_TRUE(mgr.Paused());
  mgr.EndPause();
  EXPECT_FALSE(mgr.Paused());
}

/////////////////////////////////////////////////
TEST(Manager, InitialTimeZero)
{
  gzecs::Manager mgr;
  ignition::common::Time simTime = mgr.SimulationTime();
  EXPECT_EQ(0, simTime.sec);
  EXPECT_EQ(0, simTime.nsec);
}

/////////////////////////////////////////////////
TEST(Manager, SetTimeNotPaused)
{
  gzecs::Manager mgr;
  ignition::common::Time simTime(1234, 5678);
  EXPECT_TRUE(mgr.SimulationTime(simTime));
  ignition::common::Time retrievedTime = mgr.SimulationTime();
  EXPECT_EQ(1234, retrievedTime.sec);
  EXPECT_EQ(5678, retrievedTime.nsec);
}

/////////////////////////////////////////////////
TEST(Manager, SetTimePaused)
{
  gzecs::Manager mgr;
  mgr.BeginPause();
  ignition::common::Time simTime(1234, 5678);
  EXPECT_FALSE(mgr.SimulationTime(simTime));
  ignition::common::Time retrievedTime = mgr.SimulationTime();
  EXPECT_EQ(0, retrievedTime.sec);
  EXPECT_EQ(0, retrievedTime.nsec);
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

