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

#include "componentizers/CZMaterial.hh"
#include "gazebo/components/Material.api.hh"
#include "gazebo/components/Material.factory.hh"
#include "gazebo/ecs/Manager.hh"


namespace gzecs = gazebo::ecs;
namespace gzcz = gazebo::componentizers;
namespace gzc = gazebo::components;


/////////////////////////////////////////////////
TEST(CZMaterial, NoMaterial)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::MaterialFactory>();
  mgr.LoadComponentizer<gzcz::CZMaterial>();

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
      "gazebo::components::Material",
      });

  EXPECT_EQ(0, entities.size());
}


/////////////////////////////////////////////////
TEST(CZMaterial, VisualWithMaterial)
{
  gzecs::Manager mgr;
  mgr.LoadComponentFactory<gzc::MaterialFactory>();
  mgr.LoadComponentizer<gzcz::CZMaterial>();

  std::string world = " \
    <sdf version='1.6'> \
      <world name='default'> \
        <physics name='asdf' type='ode'> \
        </physics> \
        <model name='m1'> \
          <link name='l1'> \
            <visual name='v1'> \
              <material> \
                <ambient>0.1 0.2 0.3 1.0</ambient> \
              </material> \
            </visual> \
          </link> \
        </model> \
      </world> \
    </sdf>";
  mgr.LoadWorld(world);
  mgr.UpdateOnce();

  auto entities = mgr.QueryEntities({
      "gazebo::components::Material",
      });

  ASSERT_EQ(1, entities.size());
  gzecs::Entity &e = mgr.Entity(*(entities.begin()));
  auto comp = e.Component<gazebo::components::Material>();
  EXPECT_TRUE(comp.Appearance().HasColor());
  EXPECT_FLOAT_EQ(0.1, comp.Appearance().Color().Red());
  EXPECT_FLOAT_EQ(0.2, comp.Appearance().Color().Green());
  EXPECT_FLOAT_EQ(0.3, comp.Appearance().Color().Blue());
  EXPECT_FLOAT_EQ(1.0, comp.Appearance().Color().Alpha());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
