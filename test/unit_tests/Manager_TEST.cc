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
#include "gazebo/ecs/EntityQuery.hh"
#include "gazebo/ecs/Manager.hh"

namespace gzecs = gazebo::ecs;


/////////////////////////////////////////////////
class TestHookSystem : public gzecs::System
{
  /// \brief String that's set on update
  public: std::string sentinel;

  public: virtual void Init(gzecs::QueryRegistrar &_registrar)
    {
      gzecs::EntityQuery q;
      q.AddComponent<TestComponent1>();
      _registrar.Register(q, std::bind(&TestHookSystem::Update, this,
            std::placeholders::_1));
      this->sentinel = "Init Ran";
    }

  public: void Update(const gzecs::EntityQuery &_result)
    {
      this->sentinel = "Update Ran";
    }
};

/////////////////////////////////////////////////
class TestHookComponentizer : public gzecs::Componentizer
{
  /// \brief String that's set on update
  public: std::string sentinel;

  public: virtual void Init()
    {
      this->sentinel = "Init Ran";
    }

  public: virtual void FromSDF(std::unique_ptr<gzecs::DataHandle> _handle,
              sdf::Element &_elem,
              const std::unordered_map<sdf::Element*, gzecs::EntityId> &_ids)
     {
       this->sentinel = "FromSDF Ran";
     }
};

/////////////////////////////////////////////////
TEST(Manager, LoadSystem)
{
  gzecs::Manager mgr;
  TestHookSystem *raw = new TestHookSystem;
  std::unique_ptr<gzecs::System> sys(dynamic_cast<gzecs::System*>(raw));
  mgr.LoadSystem("Test system", std::move(sys));
  EXPECT_EQ(std::string("Init Ran"), raw->sentinel);
  mgr.UpdateOnce();
  EXPECT_EQ(std::string("Update Ran"), raw->sentinel);
}

/////////////////////////////////////////////////
TEST(Manager, LoadComponentizer)
{
  gzecs::Manager mgr;
  TestHookComponentizer *raw = new TestHookComponentizer;
  std::unique_ptr<gzecs::Componentizer> cz(
      dynamic_cast<gzecs::Componentizer*>(raw));
  mgr.LoadComponentizer(std::move(cz));
  EXPECT_EQ(std::string("Init Ran"), raw->sentinel);

  mgr.LoadWorldFromSDFString(
      "<sdf version='1.6'><world name='default'></world></sdf>");
  EXPECT_EQ(std::string("FromSDF Ran"), raw->sentinel);
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
TEST(Manager, PauseCountAlwaysPositive)
{
  gzecs::Manager mgr;
  for (int i = 0; i < 10; ++i)
    EXPECT_EQ(0, mgr.EndPause());
}

/////////////////////////////////////////////////
TEST(Manager, InitiallyNotPaused)
{
  gzecs::Manager mgr;
  EXPECT_FALSE(mgr.Paused());
  mgr.UpdateOnce();
  EXPECT_FALSE(mgr.Paused());
}

/////////////////////////////////////////////////
TEST(Manager, PauseUnpause)
{
  gzecs::Manager mgr;
  mgr.BeginPause();
  mgr.UpdateOnce();
  EXPECT_TRUE(mgr.Paused());
  mgr.EndPause();
  mgr.UpdateOnce();
  EXPECT_FALSE(mgr.Paused());
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

