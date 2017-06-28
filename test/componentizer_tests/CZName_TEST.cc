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

#include "componentizers/CZName.hh"
#include "gazebo/components/Name.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;


/////////////////////////////////////////////////
TEST(CZName, RegisterComponent)
{
  gzcz::CZName cz;
  cz.Init();
  gzecs::ComponentType t =
    gzecs::ComponentFactory::Type<gazebo::components::Name>();
  ASSERT_NE(gzecs::NO_COMPONENT, t);
  gzecs::ComponentTypeInfo info = gzecs::ComponentFactory::TypeInfo(t);
  EXPECT_EQ("gazebo::components::Name", info.name);
}

/////////////////////////////////////////////////
TEST(CZName, SdfEmptyWorld)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZName>();

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
      "gazebo::components::Name",
      });

  EXPECT_EQ(2, entities.size());
}


/////////////////////////////////////////////////
TEST(CZName, SdfWithStuff)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZName>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='some_model'> \
          <link name='some_link'> \
            <collision name='some_collision'> \
            </collision> \
            <visual name='some_visual'> \
            </visual> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Name",
      });

  EXPECT_EQ(6, entities.size());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
