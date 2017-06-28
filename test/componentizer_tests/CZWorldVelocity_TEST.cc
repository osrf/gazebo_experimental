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

#include "componentizers/CZWorldVelocity.hh"
#include "gazebo/components/WorldVelocity.api.hh"
#include "gazebo/components/WorldVelocity.factory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;
namespace gzc = gazebo::components;


/////////////////////////////////////////////////
TEST(CZWorldVelocity, NoWorldVelocity)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::WorldVelocityFactory>();
  mgr.LoadComponentizer<gzcz::CZWorldVelocity>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::WorldVelocity",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZWorldVelocity, MovingModel)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::WorldVelocityFactory>();
  mgr.LoadComponentizer<gzcz::CZWorldVelocity>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::WorldVelocity",
      });

  EXPECT_EQ(1, entities.size());
}

/////////////////////////////////////////////////
TEST(CZWorldVelocity, StaticModel)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::WorldVelocityFactory>();
  mgr.LoadComponentizer<gzcz::CZWorldVelocity>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <static>true</static> \
          <link name='l1'> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::WorldVelocity",
      });

  EXPECT_EQ(0, entities.size());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
