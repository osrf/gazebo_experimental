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

#include "componentizers/CZPose.hh"
#include "gazebo/components/Pose.api.hh"
#include "gazebo/components/Pose.factory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;
namespace gzc = gazebo::components;

// Euler -> Quaternion -> Euler needs a tiny bit more allowed error
const double allowedAngularError = 1e-15;


/////////////////////////////////////////////////
TEST(CZPose, NoPose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  mgr.LoadComponentizer<gzcz::CZPose>();

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
      "gazebo::components::Pose",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZPose, ModelWithPose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  mgr.LoadComponentizer<gzcz::CZPose>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <pose>1.1 2.2 3.3 0.1 1.2 2.3</pose> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Pose",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Pose>();
  EXPECT_DOUBLE_EQ(1.1, comp.Origin().Pos().X());
  EXPECT_DOUBLE_EQ(2.2, comp.Origin().Pos().Y());
  EXPECT_DOUBLE_EQ(3.3, comp.Origin().Pos().Z());

  auto eulerAngles = comp.Origin().Rot().Euler();
  EXPECT_NEAR(0.1, eulerAngles.X(), allowedAngularError);
  EXPECT_NEAR(1.2, eulerAngles.Y(), allowedAngularError);
  EXPECT_NEAR(2.3, eulerAngles.Z(), allowedAngularError);
}

/////////////////////////////////////////////////
TEST(CZPose, ManyPoses)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  mgr.LoadComponentizer<gzcz::CZPose>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <collision name='c1'> \
            </collision> \
            <visual name='v1'> \
            </visual> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorldFromString(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Pose",
      });

  ASSERT_EQ(4, entities.size());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
