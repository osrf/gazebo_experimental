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

#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"


// Component Types for testing
struct TC1
{
  float itemOne;
};

struct TC2
{
  float itemOne;
  int itemTwo;
};

struct TC3
{
  float itemOne;
  int itemTwo;
  double itemThree;
};

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, FirstEntityIdIsNotNoEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();
  EXPECT_NE(gazebo::ecs::NO_ENTITY, entity);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, CreateUniqueEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;

  for (int i = 0; i < 100; i++)
  {
    gazebo::ecs::EntityId e = uut.CreateEntity();
    EXPECT_EQ(entities.end(), std::find(entities.begin(), entities.end(), e));
    entities.push_back(e);
  }
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddComponentToOneEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[2]);
  uut.Update();

  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[1]));
  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(entities[2]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddDifferentComponentsToDifferentEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();
  auto const type3 = gazebo::ecs::ComponentFactory::Type<TC3>();

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);
  uut.Update();

  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[1]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[2]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[3]));

  EXPECT_EQ(nullptr, uut.EntityComponent<TC2>(entities[0]));
  EXPECT_NE(nullptr, uut.EntityComponent<TC2>(entities[1]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC2>(entities[2]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC2>(entities[3]));

  EXPECT_EQ(nullptr, uut.EntityComponent<TC3>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC3>(entities[1]));
  EXPECT_NE(nullptr, uut.EntityComponent<TC3>(entities[2]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC3>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddMultipleComponentsToOneEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC3>(entities[0]);
  uut.Update();

  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[1]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entities[2]));

  EXPECT_NE(nullptr, uut.EntityComponent<TC2>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC2>(entities[1]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC2>(entities[2]));

  EXPECT_NE(nullptr, uut.EntityComponent<TC3>(entities[0]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC3>(entities[1]));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC3>(entities[2]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreEditableWhenAdding)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;

  auto first = uut.AddComponent<TC1>(entity);
  first->itemOne = one_one;
  auto second = uut.AddComponent<TC2>(entity);
  second->itemOne = two_one;
  second->itemTwo = two_two;
  auto third = uut.AddComponent<TC3>(entity);
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;

  uut.Update();

  ASSERT_NE(nullptr, uut.EntityComponent<TC1>(entity));
  ASSERT_NE(nullptr, uut.EntityComponent<TC2>(entity));
  ASSERT_NE(nullptr, uut.EntityComponent<TC3>(entity));

  EXPECT_EQ(one_one, uut.EntityComponent<TC1>(entity)->itemOne);
  EXPECT_EQ(two_one, uut.EntityComponent<TC2>(entity)->itemOne);
  EXPECT_EQ(two_two, uut.EntityComponent<TC2>(entity)->itemTwo);
  EXPECT_EQ(three_one, uut.EntityComponent<TC3>(entity)->itemOne);
  EXPECT_EQ(three_two, uut.EntityComponent<TC3>(entity)->itemTwo);
  EXPECT_EQ(three_three, uut.EntityComponent<TC3>(entity)->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreNotEditableBeforeFirstUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  auto first = uut.AddComponent<TC1>(entity);

  ASSERT_EQ(nullptr, uut.EntityComponent<TC1>(entity));
  ASSERT_EQ(nullptr, uut.EntityComponentMutable<TC1>(entity));
  uut.Update();
  ASSERT_NE(nullptr, uut.EntityComponent<TC1>(entity));
  ASSERT_NE(nullptr, uut.EntityComponentMutable<TC1>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreEditableAfterFirstUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  uut.AddComponent<TC1>(entity);
  uut.AddComponent<TC2>(entity);
  uut.AddComponent<TC3>(entity);
  uut.Update();

  float one_one = 123.0f;
  float two_one = 456.0f;
  int two_two = 7;
  float three_one = 789.0f;
  float three_two = 9;
  float three_three = -147.8f;

  auto first = uut.EntityComponentMutable<TC1>(entity);
  auto second = uut.EntityComponentMutable<TC2>(entity);
  auto third = uut.EntityComponentMutable<TC3>(entity);

  ASSERT_NE(nullptr, first);
  ASSERT_NE(nullptr, second);
  ASSERT_NE(nullptr, third);

  first->itemOne = one_one;
  second->itemOne = two_one;
  second->itemTwo = two_two;
  third->itemOne = three_one;
  third->itemTwo = three_two;
  third->itemThree = three_three;
  uut.Update();

  EXPECT_EQ(one_one, uut.EntityComponent<TC1>(entity)->itemOne);
  EXPECT_EQ(two_one, uut.EntityComponent<TC2>(entity)->itemOne);
  EXPECT_EQ(two_two, uut.EntityComponent<TC2>(entity)->itemTwo);
  EXPECT_EQ(three_one, uut.EntityComponent<TC3>(entity)->itemOne);
  EXPECT_EQ(three_two, uut.EntityComponent<TC3>(entity)->itemTwo);
  EXPECT_EQ(three_three, uut.EntityComponent<TC3>(entity)->itemThree);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueryExistingEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[0]);
  uut.AddComponent<TC3>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);
  uut.Update();

  gazebo::ecs::EntityQuery query;
  query.AddComponent("TC2");
  query.AddComponent("TC3");
  auto queryAdd = uut.AddQuery(std::move(query));

  ASSERT_EQ(2, uut.Query(queryAdd.first).EntityIds().size());
  bool foundE1 = false;
  bool foundE2 = false;
  bool foundE3 = false;
  for (auto result : uut.Query(queryAdd.first).EntityIds())
  {
    if (result == entities[0])
      foundE1 = true;
    else if (result == entities[1])
      foundE2 = true;
    else
      foundE3 = true;
  }

  EXPECT_TRUE(foundE1);
  EXPECT_TRUE(foundE2);
  EXPECT_FALSE(foundE3);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueryNewEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;

  gazebo::ecs::EntityQuery query;
  query.AddComponent("TC2");
  query.AddComponent("TC3");
  auto queryAdd = uut.AddQuery(std::move(query));

  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[0]);
  uut.AddComponent<TC3>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);

  EXPECT_EQ(0, uut.Query(queryAdd.first).EntityIds().size());
  uut.Update();
  EXPECT_EQ(2, uut.Query(queryAdd.first).EntityIds().size());

  bool foundE1 = false;
  bool foundE2 = false;
  bool foundE3 = false;
  for (auto result : uut.Query(queryAdd.first).EntityIds())
  {
    if (result == entities[0])
      foundE1 = true;
    else if (result == entities[1])
      foundE2 = true;
    else
      foundE3 = true;
  }

  EXPECT_TRUE(foundE1);
  EXPECT_TRUE(foundE2);
  EXPECT_FALSE(foundE3);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveQueryNoResults)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC2>(entities[0]);
  uut.AddComponent<TC2>(entities[1]);
  uut.AddComponent<TC3>(entities[0]);
  uut.AddComponent<TC3>(entities[1]);
  uut.AddComponent<TC3>(entities[2]);

  gazebo::ecs::EntityQuery query;
  query.AddComponent("TC2");
  query.AddComponent("TC3");

  auto result = uut.AddQuery(std::move(query));
  uut.RemoveQuery(result.first);

  ASSERT_EQ(0, uut.Query(result.first).EntityIds().size());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, TrackComponentChanges)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity = uut.CreateEntity();

  uut.AddComponent<TC1>(entity);
  // component is created after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC1>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TC1>(entity));

  uut.AddComponent<TC2>(entity);
  // component is created after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TC2>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC1>(entity));

  uut.Update();
  uut.RemoveComponent<TC1>(entity);
  // component is removed after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC1>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TC1>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC2>(entity));

  uut.DeleteEntity(entity);
  // component and entity are removed after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TC2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TC2>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueriesIncludeDeletedEntitiesForOneUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TC1>(entities[0]);
  uut.AddComponent<TC1>(entities[1]);
  uut.AddComponent<TC1>(entities[2]);


  gazebo::ecs::EntityQuery query;
  query.AddComponent("TC1");

  auto result = uut.AddQuery(std::move(query));
  uut.Update();
  EXPECT_EQ(3, uut.Query(result.first).EntityIds().size());

  uut.DeleteEntity(entities[0]);
  uut.DeleteEntity(entities[1]);
  uut.DeleteEntity(entities[2]);

  uut.Update();
  EXPECT_EQ(3, uut.Query(result.first).EntityIds().size());
  for (auto id : uut.Query(result.first).EntityIds())
  {
    EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TC1>(id));
  }

  uut.Update();
  EXPECT_EQ(0, uut.Query(result.first).EntityIds().size());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ModifyThenRemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  uut.AddComponent<TC1>(entity);
  uut.Update();

  ASSERT_NE(nullptr, uut.EntityComponentMutable<TC1>(entity));
  EXPECT_TRUE(uut.RemoveComponent<TC1>(entity));

  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TC1>(entity));
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, CreateComponentSetInitialValues)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TC1>(entity);
  comp->itemOne = 12345.0;

  uut.Update();
  auto constComp = uut.EntityComponent<TC1>(entity);
  ASSERT_NE(nullptr, constComp);
  EXPECT_FLOAT_EQ(12345.0, comp->itemOne);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, MutableComponentHasSetValues)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TC1>(entity);
  comp->itemOne = 12345.0;
  uut.Update();

  comp = uut.EntityComponentMutable<TC1>(entity);
  ASSERT_NE(nullptr, comp);
  EXPECT_FLOAT_EQ(12345.0, comp->itemOne);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ModifyComponentHappensNextUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TC1>(entity);
  comp->itemOne = 12345.0;
  uut.Update();

  comp = uut.EntityComponentMutable<TC1>(entity);
  ASSERT_NE(nullptr, comp);
  comp->itemOne = 6789.0;

  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TC1>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_MODIFIED, uut.IsDifferent<TC1>(entity));

  auto constComp = uut.EntityComponent<TC1>(entity);
  ASSERT_NE(nullptr, constComp);
  EXPECT_FLOAT_EQ(6789.0, constComp->itemOne);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId id = uut.CreateEntity();

  uut.AddComponent<TC1>(id);
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(id));
  uut.Update();
  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(id));

  uut.RemoveComponent<TC1>(id);
  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(id));
  uut.Update();
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(id));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveComponentAccessAnother)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId id = uut.CreateEntity();
  auto const type1 = gazebo::ecs::ComponentFactory::Type<TC1>();
  auto const type2 = gazebo::ecs::ComponentFactory::Type<TC2>();

  auto c1 = uut.AddComponent<TC1>(id);
  auto c2 = uut.AddComponent<TC2>(id);
  ASSERT_NE(nullptr, c1);
  ASSERT_NE(nullptr, c2);

  c1->itemOne = 1234;
  c2->itemOne = 5678;
  c2->itemTwo = 9012;

  uut.Update();
  uut.RemoveComponent<TC1>(id);

  auto c2_test = uut.EntityComponent<TC2>(id);
  ASSERT_NE(nullptr, c2_test);
  EXPECT_EQ(5678, c2_test->itemOne);
  EXPECT_EQ(9012, c2_test->itemTwo);

  uut.Update();
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(id));
  c2_test = uut.EntityComponent<TC2>(id);
  ASSERT_NE(nullptr, c2_test);
  EXPECT_EQ(5678, c2_test->itemOne);
  EXPECT_EQ(9012, c2_test->itemTwo);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, DeleteEntity)
{
  gazebo::ecs::EntityComponentDatabase uut;
  gazebo::ecs::EntityId id = uut.CreateEntity();

  uut.AddComponent<TC1>(id);
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(id));
  uut.Update();
  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(id));

  uut.DeleteEntity(id);
  EXPECT_NE(nullptr, uut.EntityComponent<TC1>(id));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::NO_ENTITY, uut.Entity(id).Id());
  EXPECT_EQ(nullptr, uut.EntityComponent<TC1>(id));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ReuseSmallestAvailableEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.Update();

  uut.DeleteEntity(entities[2]);
  uut.DeleteEntity(entities[3]);
  // deleted ids not available yet
  entities.push_back(uut.CreateEntity());
  EXPECT_EQ(5, entities.back());

  uut.Update();
  // deleted ids still not availble
  entities.push_back(uut.CreateEntity());
  EXPECT_EQ(6, entities.back());

  uut.Update();
  // Now the deleted ids can be reused
  EXPECT_EQ(2, uut.CreateEntity());
  EXPECT_EQ(3, uut.CreateEntity());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, IsDifferent)
{
  gazebo::ecs::EntityComponentDatabase db;
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  entity.AddComponent<TC1>();
  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, entity.IsDifferent<TC1>());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, ReadComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  auto comp = entity.AddComponent<TC1>();
  comp->itemOne = 1234.0;
  db.Update();
  auto readOnlyComp = entity.Component<TC1>();
  EXPECT_FLOAT_EQ(1234.0, readOnlyComp->itemOne);

  db.Update();
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, entity.IsDifferent<TC1>());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, WriteComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  auto comp = entity.AddComponent<TC1>();
  comp->itemOne = 1234.0;
  db.Update();
  auto mutableComp = entity.ComponentMutable<TC1>();
  mutableComp->itemOne = 4567.0;

  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_MODIFIED, entity.IsDifferent<TC1>());
  auto readOnlyComp = entity.Component<TC1>();
  EXPECT_FLOAT_EQ(4567.0, readOnlyComp->itemOne);
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, RemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  entity.AddComponent<TC1>();
  db.Update();
  entity.RemoveComponent<TC1>();
  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, entity.IsDifferent<TC1>());
}

int main(int argc, char **argv)
{
  gazebo::ecs::ComponentFactory::Register<TC1>("TC1");
  gazebo::ecs::ComponentFactory::Register<TC2>("TC2");
  gazebo::ecs::ComponentFactory::Register<TC3>("TC3");

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
