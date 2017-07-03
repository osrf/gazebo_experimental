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
#include "gazebo/components/PoseHelper.hh"
#include "gazebo/components/Pose.api.hh"
#include "gazebo/components/Pose.factory.hh"

namespace gzecs = gazebo::ecs;
namespace gzc = gazebo::components;

const double tolerance = 1e-15;

/////////////////////////////////////////////////
TEST(PoseHelper, AbsolutePose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id = mgr.CreateEntity();
  auto &e = mgr.Entity(id);
  e.AddComponent<gzc::Pose>();
  mgr.UpdateOnce();

  ignition::math::Pose3d pose(1.0, 2.0, 3.0, .4, .5, .6);
  EXPECT_TRUE(gzc::SetWorldPose(mgr, e, pose));
  mgr.UpdateOnce();

  ignition::math::Pose3d uutPose;
  EXPECT_TRUE(gzc::WorldPose(mgr, e, uutPose));
  EXPECT_DOUBLE_EQ(pose.Pos().X(), uutPose.Pos().X());
  EXPECT_DOUBLE_EQ(pose.Pos().Y(), uutPose.Pos().Y());
  EXPECT_DOUBLE_EQ(pose.Pos().Z(), uutPose.Pos().Z());
  EXPECT_DOUBLE_EQ(pose.Rot().W(), uutPose.Rot().W());
  EXPECT_DOUBLE_EQ(pose.Rot().X(), uutPose.Rot().X());
  EXPECT_DOUBLE_EQ(pose.Rot().Y(), uutPose.Rot().Y());
  EXPECT_DOUBLE_EQ(pose.Rot().Z(), uutPose.Rot().Z());
}


/////////////////////////////////////////////////
TEST(PoseHelper, RelativePose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id1 = mgr.CreateEntity();
  gzecs::EntityId id2 = mgr.CreateEntity();
  auto &e1 = mgr.Entity(id1);
  auto &e2 = mgr.Entity(id2);
  auto p1 = e1.AddComponent<gzc::Pose>();
  auto p2 = e2.AddComponent<gzc::Pose>();
  ignition::math::Pose3d p1InitialTransform(1.0, 2.0, 3.0, M_PI, 0.0, 0.0);
  ignition::math::Pose3d p2InitialTransform(3.0, 2.0, 1.0, 0.0, 0.0, M_PI/2.0);
  p1.Transform() = p1InitialTransform;
  p2.Transform() = p2InitialTransform;
  p2.AttachedTo() = id1;
  mgr.UpdateOnce();

  ignition::math::Pose3d goldenPose(4.0, 0.0, 2.0, M_PI, 0.0, -M_PI/2.0);
  ignition::math::Pose3d uutPose;
  EXPECT_TRUE(gzc::WorldPose(mgr, e2, uutPose));
  EXPECT_DOUBLE_EQ(goldenPose.Pos().X(), uutPose.Pos().X());
  EXPECT_DOUBLE_EQ(goldenPose.Pos().Y(), uutPose.Pos().Y());
  EXPECT_DOUBLE_EQ(goldenPose.Pos().Z(), uutPose.Pos().Z());
  EXPECT_NEAR(goldenPose.Rot().Roll(), uutPose.Rot().Roll(), tolerance);
  EXPECT_NEAR(goldenPose.Rot().Pitch(), uutPose.Rot().Pitch(), tolerance);
  EXPECT_NEAR(goldenPose.Rot().Yaw(), uutPose.Rot().Yaw(), tolerance);

  EXPECT_TRUE(gzc::SetWorldPose(mgr, e2, p2InitialTransform));
  mgr.UpdateOnce();

  goldenPose = p2InitialTransform;
  EXPECT_TRUE(gzc::WorldPose(mgr, e2, uutPose));
  EXPECT_DOUBLE_EQ(goldenPose.Pos().X(), uutPose.Pos().X());
  EXPECT_DOUBLE_EQ(goldenPose.Pos().Y(), uutPose.Pos().Y());
  EXPECT_DOUBLE_EQ(goldenPose.Pos().Z(), uutPose.Pos().Z());
  EXPECT_NEAR(goldenPose.Rot().Roll(), uutPose.Rot().Roll(), tolerance);
  EXPECT_NEAR(goldenPose.Rot().Pitch(), uutPose.Rot().Pitch(), tolerance);
  EXPECT_NEAR(goldenPose.Rot().Yaw(), uutPose.Rot().Yaw(), tolerance);
}

/////////////////////////////////////////////////
TEST(PoseHelper, AttachedNonexistantEntity)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id = mgr.CreateEntity();
  auto &e = mgr.Entity(id);
  auto p = e.AddComponent<gzc::Pose>();
  p.AttachedTo() = 123456789;
  mgr.UpdateOnce();

  ignition::math::Pose3d uutPose;
  EXPECT_FALSE(gzc::WorldPose(mgr, e, uutPose));
}

/////////////////////////////////////////////////
TEST(PoseHelper, AttachedToRelativeEntity)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id1 = mgr.CreateEntity();
  gzecs::EntityId id2 = mgr.CreateEntity();
  auto &e1 = mgr.Entity(id1);
  auto &e2 = mgr.Entity(id2);
  auto p1 = e1.AddComponent<gzc::Pose>();
  auto p2 = e2.AddComponent<gzc::Pose>();
  p1.AttachedTo() = id2;
  p2.AttachedTo() = id1;
  mgr.UpdateOnce();

  ignition::math::Pose3d uutPose;
  EXPECT_FALSE(gzc::WorldPose(mgr, e1, uutPose));
  EXPECT_FALSE(gzc::WorldPose(mgr, e2, uutPose));
}

/////////////////////////////////////////////////
TEST(PoseHelper, AttachedEntityNoPose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id1 = mgr.CreateEntity();
  gzecs::EntityId id2 = mgr.CreateEntity();
  auto &e2 = mgr.Entity(id2);
  auto p2 = e2.AddComponent<gzc::Pose>();
  p2.AttachedTo() = id1;
  mgr.UpdateOnce();

  ignition::math::Pose3d uutPose;
  EXPECT_FALSE(gzc::WorldPose(mgr, e2, uutPose));
}

/////////////////////////////////////////////////
TEST(PoseHelper, NoPose)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::PoseFactory>();
  gzecs::EntityId id = mgr.CreateEntity();
  auto &e = mgr.Entity(id);
  mgr.UpdateOnce();

  ignition::math::Pose3d uutPose;
  EXPECT_FALSE(gzc::WorldPose(mgr, e, uutPose));
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
