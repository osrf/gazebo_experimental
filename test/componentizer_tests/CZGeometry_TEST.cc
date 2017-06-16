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

#include "componentizers/CZGeometry.hh"
#include "gazebo/components/Geometry.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;


/////////////////////////////////////////////////
TEST(CZGeometry, RegisterComponent)
{
  gzcz::CZGeometry cz;
  cz.Init();
  gzecs::ComponentType t =
    gzecs::ComponentFactory::Type<gazebo::components::Geometry>();
  ASSERT_NE(gzecs::NO_COMPONENT, t);
  gzecs::ComponentTypeInfo info = gzecs::ComponentFactory::TypeInfo(t);
  EXPECT_EQ("gazebo::components::Geometry", info.name);
}


/////////////////////////////////////////////////
TEST(CZGeometry, SdfNoGeometry)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZGeometry>();

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
      "gazebo::components::Geometry",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZGeometry, SdfSphereGeometry)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZGeometry>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <collision name='c1'> \
              <geometry> \
                <sphere> \
                  <radius>0.5</radius> \
                </sphere> \
              </geometry> \
            </collision> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Geometry",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Geometry>();
  EXPECT_EQ(gazebo::components::Geometry::SPHERE, comp->type);
  EXPECT_DOUBLE_EQ(0.5, comp->sphere.radius);
}

/////////////////////////////////////////////////
TEST(CZGeometry, SdfBoxGeometry)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZGeometry>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <collision name='c1'> \
              <geometry> \
                <box> \
                  <size>0.5 0.6 0.7</size> \
                </box> \
              </geometry> \
            </collision> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Geometry",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Geometry>();
  EXPECT_EQ(gazebo::components::Geometry::BOX, comp->type);
  EXPECT_DOUBLE_EQ(0.5, comp->box.size.X());
  EXPECT_DOUBLE_EQ(0.6, comp->box.size.Y());
  EXPECT_DOUBLE_EQ(0.7, comp->box.size.Z());
}

/////////////////////////////////////////////////
TEST(CZGeometry, SdfCylinderGeometry)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZGeometry>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <collision name='c1'> \
              <geometry> \
                <cylinder> \
                  <radius>0.54321</radius> \
                  <length>1.234</length> \
                </cylinder> \
              </geometry> \
            </collision> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Geometry",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Geometry>();
  EXPECT_EQ(gazebo::components::Geometry::CYLINDER, comp->type);
  EXPECT_DOUBLE_EQ(0.54321, comp->cylinder.radius);
  EXPECT_DOUBLE_EQ(1.234, comp->cylinder.length);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

