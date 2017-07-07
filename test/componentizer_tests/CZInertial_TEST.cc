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

#include "componentizers/CZInertial.hh"
#include "gazebo/components/Inertial.api.hh"
#include "gazebo/components/Inertial.factory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;
namespace gzc = gazebo::components;


/////////////////////////////////////////////////
TEST(CZInertial, NoInertial)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::InertialFactory>();
  mgr.LoadComponentizer<gzcz::CZInertial>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromSDFString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Inertial",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZInertial, LinkWithInertial)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::InertialFactory>();
  mgr.LoadComponentizer<gzcz::CZInertial>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <inertial> \
              <mass>3.14</mass> \
              <inertia> \
                <ixx>1.23</ixx> \
                <iyy>2.34</iyy> \
                <izz>3.45</izz> \
                <ixy>4.56</ixy> \
                <ixz>5.67</ixz> \
                <iyz>6.78</iyz> \
              </inertia> \
            </inertial> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromSDFString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Inertial",
      });

  ASSERT_EQ(1, entities.size());
  auto handle = mgr.Handle();
  gzecs::Entity &e = handle->Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Inertial>();
  EXPECT_DOUBLE_EQ(3.14, comp.Mass());
  EXPECT_DOUBLE_EQ(1.23, comp.Inertia()(0, 0));
  EXPECT_DOUBLE_EQ(2.34, comp.Inertia()(1, 1));
  EXPECT_DOUBLE_EQ(3.45, comp.Inertia()(2, 2));
  EXPECT_DOUBLE_EQ(4.56, comp.Inertia()(0, 1));
  EXPECT_DOUBLE_EQ(4.56, comp.Inertia()(1, 0));
  EXPECT_DOUBLE_EQ(5.67, comp.Inertia()(0, 2));
  EXPECT_DOUBLE_EQ(5.67, comp.Inertia()(2, 0));
  EXPECT_DOUBLE_EQ(6.78, comp.Inertia()(1, 2));
  EXPECT_DOUBLE_EQ(6.78, comp.Inertia()(2, 1));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
