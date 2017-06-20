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
#define GAZEBO_TESTHOOK 1

#include "componentizers/CZPhysicsConfig.hh"
#include "gazebo/components/PhysicsConfig.api.hh"
#include "gazebo/components/PhysicsConfig.factory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;
namespace gzc = gazebo::components;


/////////////////////////////////////////////////
TEST(CZPhysicsConfig, SdfOneConfig)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PhysicsConfigFactory>();
  mgr.LoadComponentizer<gzcz::CZPhysicsConfig>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::PhysicsConfig",
      });

  EXPECT_EQ(1, entities.size());
}


/////////////////////////////////////////////////
TEST(CZPhysicsConfig, SdfTwoConfigs)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PhysicsConfigFactory>();
  mgr.LoadComponentizer<gzcz::CZPhysicsConfig>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <physics name='ghjk' type='ode'> \
        </physics> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::PhysicsConfig",
      });

  EXPECT_EQ(2, entities.size());
}


/////////////////////////////////////////////////
TEST(CZPhysicsConfig, SdfMaxStepSize)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PhysicsConfigFactory>();
  mgr.LoadComponentizer<gzcz::CZPhysicsConfig>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
          <max_step_size>1.234</max_step_size> \
        </physics> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::PhysicsConfig",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::PhysicsConfig>();
  EXPECT_DOUBLE_EQ(1.234, comp.MaxStepSize());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
