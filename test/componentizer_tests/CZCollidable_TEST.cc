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

#include "componentizers/CZCollidable.hh"
#include "gazebo/components/Collidable.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;


/////////////////////////////////////////////////
TEST(CZCollidable, RegisterComponent)
{
  gzcz::CZCollidable cz;
  cz.Init();
  gzecs::ComponentType t =
    gzecs::ComponentFactory::Type<gazebo::components::Collidable>();
  ASSERT_NE(gzecs::NO_COMPONENT, t);
  gzecs::ComponentTypeInfo info = gzecs::ComponentFactory::TypeInfo(t);
  EXPECT_EQ("gazebo::components::Collidable", info.name);
}


/////////////////////////////////////////////////
TEST(CZCollidable, SdfNoCollisions)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZCollidable>();

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
      "gazebo::components::Collidable",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZCollidable, SdfOneCollision)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZCollidable>();

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
      "gazebo::components::Collidable",
      });

  EXPECT_EQ(1, entities.size());
}

/////////////////////////////////////////////////
TEST(CZCollidable, SdfTwoCollisions)
{
  gzecs::Manager mgr;
  mgr.LoadComponentizer<gzcz::CZCollidable>();

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
          <collision name='c2'> \
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
      "gazebo::components::Collidable",
      });

  ASSERT_EQ(2, entities.size());
  gzecs::Entity &e1 = mgr.Entity(*(entities.begin()));
  gzecs::Entity &e2 = mgr.Entity(*(++entities.begin()));
  auto comp1 = e1.Component<gazebo::components::Collidable>();
  auto comp2 = e2.Component<gazebo::components::Collidable>();
  EXPECT_NE(gzecs::NO_ENTITY, comp1->groupId);
  EXPECT_EQ(comp1->groupId, comp2->groupId);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
