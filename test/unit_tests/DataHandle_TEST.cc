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
#include "util/TestComponent.hh"
#include "gazebo/ecs/Manager.hh"

namespace gzecs = gazebo::ecs;

/////////////////////////////////////////////////
TEST(DataHandle, CreateEntity)
{
  gzecs::Manager mgr;
  auto handle = mgr.Handle();
  gzecs::EntityId id = handle->CreateEntity();
  EXPECT_NE(gzecs::NO_ENTITY, id);
}

/////////////////////////////////////////////////
TEST(DataHandle, GetEntity)
{
  gzecs::Manager mgr;
  auto handle = mgr.Handle();
  gzecs::EntityId id = handle->CreateEntity();
  gzecs::Entity &entity = handle->Entity(id);
  EXPECT_EQ(id, entity.Id());
}

/////////////////////////////////////////////////
TEST(DataHandle, DeleteEntity)
{
  gzecs::Manager mgr;
  gzecs::EntityId id;
  {
    auto handle = mgr.Handle();
    id = handle->CreateEntity();
  }
  mgr.UpdateOnce();
  {
    auto handle = mgr.Handle();
    handle->DeleteEntity(id);
  }
  mgr.UpdateOnce();
  auto handle = mgr.Handle();
  EXPECT_EQ(gzecs::NO_ENTITY, handle->Entity(id).Id());
}

/////////////////////////////////////////////////
TEST(DataHandle, InitialTimeZero)
{
  gzecs::Manager mgr;
  auto handle = mgr.Handle();
  ignition::common::Time simTime = handle->SimulationTime();
  EXPECT_EQ(0, simTime.sec);
  EXPECT_EQ(0, simTime.nsec);
}

/////////////////////////////////////////////////
TEST(DataHandle, SetTimeNotPaused)
{
  gzecs::Manager mgr;
  ignition::common::Time simTime(1234, 5678);
  {
    auto handle = mgr.Handle();
    EXPECT_TRUE(handle->SimulationTime(simTime));
  }

  mgr.UpdateOnce();

  auto handle = mgr.Handle();
  simTime = handle->SimulationTime();

  EXPECT_EQ(1234, simTime.sec);
  EXPECT_EQ(5678, simTime.nsec);
}

/////////////////////////////////////////////////
TEST(DataHandle, SetTimeNextUpdate)
{
  gzecs::Manager mgr;
  ignition::common::Time simTime(1234, 5678);
  {
    auto handle = mgr.Handle();
    EXPECT_TRUE(handle->SimulationTime(simTime));

    simTime = handle->SimulationTime();
  }
  EXPECT_EQ(0, simTime.sec);
  EXPECT_EQ(0, simTime.nsec);

  mgr.UpdateOnce();

  auto handle = mgr.Handle();
  simTime = handle->SimulationTime();

  EXPECT_EQ(1234, simTime.sec);
  EXPECT_EQ(5678, simTime.nsec);
}

/////////////////////////////////////////////////
TEST(DataHandle, SetTimePaused)
{
  gzecs::Manager mgr;
  mgr.BeginPause();
  mgr.UpdateOnce();

  ignition::common::Time simTime(1234, 5678);
  {
    auto handle = mgr.Handle();
    EXPECT_FALSE(handle->SimulationTime(simTime));
  }

  mgr.UpdateOnce();

  auto handle = mgr.Handle();
  simTime = handle->SimulationTime();
  EXPECT_EQ(0, simTime.sec);
  EXPECT_EQ(0, simTime.nsec);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


