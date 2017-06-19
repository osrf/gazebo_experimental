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

#include "util/TestComponent.hh"
#include "gazebo/ecs/EntityComponentDatabase.hh"
#include "gazebo/ecs/EntityQuery.hh"


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
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));

  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[2]);
  uut.Update();

  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[1]));
  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(entities[2]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddDifferentComponentsToDifferentEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[2]);
  uut.Update();

  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[1]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[2]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[3]));

  EXPECT_FALSE(uut.EntityComponent<TestComponent2>(entities[0]));
  EXPECT_TRUE(uut.EntityComponent<TestComponent2>(entities[1]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent2>(entities[2]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent2>(entities[3]));

  EXPECT_FALSE(uut.EntityComponent<TestComponent3>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent3>(entities[1]));
  EXPECT_TRUE(uut.EntityComponent<TestComponent3>(entities[2]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent3>(entities[3]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, AddMultipleComponentsToOneEntityId)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[0]);
  uut.AddComponent<TestComponent3>(entities[0]);
  uut.Update();

  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[1]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entities[2]));

  EXPECT_TRUE(uut.EntityComponent<TestComponent2>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent2>(entities[1]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent2>(entities[2]));

  EXPECT_TRUE(uut.EntityComponent<TestComponent3>(entities[0]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent3>(entities[1]));
  EXPECT_FALSE(uut.EntityComponent<TestComponent3>(entities[2]));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreEditableWhenAdding)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  int c1_1 = 34431;
  int c1_2 = -432;
  int c2_1 = -9701;
  int c2_2 = -14981;
  int c3_1 = 1811;
  int c3_2 = 1712;

  auto first = uut.AddComponent<TestComponent1>(entity);
  auto second = uut.AddComponent<TestComponent2>(entity);
  auto third = uut.AddComponent<TestComponent3>(entity);

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_TRUE(third);

  first.Item1() = c1_1;
  first.Item2() = c1_2;
  second.Item1() = c2_1;
  second.Item2() = c2_2;
  third.Item1() = c3_1;
  third.Item2() = c3_2;

  uut.Update();

  ASSERT_TRUE(uut.EntityComponent<TestComponent1>(entity));
  ASSERT_TRUE(uut.EntityComponent<TestComponent2>(entity));
  ASSERT_TRUE(uut.EntityComponent<TestComponent3>(entity));

  EXPECT_EQ(c1_1, uut.EntityComponent<TestComponent1>(entity).Item1());
  EXPECT_EQ(c1_2, uut.EntityComponent<TestComponent1>(entity).Item2());
  EXPECT_EQ(c2_1, uut.EntityComponent<TestComponent2>(entity).Item1());
  EXPECT_EQ(c2_2, uut.EntityComponent<TestComponent2>(entity).Item2());
  EXPECT_EQ(c3_1, uut.EntityComponent<TestComponent3>(entity).Item1());
  EXPECT_EQ(c3_2, uut.EntityComponent<TestComponent3>(entity).Item2());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreNotEditableBeforeFirstUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  auto first = uut.AddComponent<TestComponent1>(entity);

  ASSERT_FALSE(uut.EntityComponent<TestComponent1>(entity));
  ASSERT_FALSE(uut.EntityComponentMutable<TestComponent1>(entity));
  uut.Update();
  ASSERT_TRUE(uut.EntityComponent<TestComponent1>(entity));
  ASSERT_TRUE(uut.EntityComponentMutable<TestComponent1>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ComponentsAreEditableAfterFirstUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  gazebo::ecs::EntityId entity;
  entity = uut.CreateEntity();

  uut.AddComponent<TestComponent1>(entity);
  uut.AddComponent<TestComponent2>(entity);
  uut.AddComponent<TestComponent3>(entity);
  uut.Update();

  int c1_1 = 34431;
  int c1_2 = -432;
  int c2_1 = -9701;
  int c2_2 = -14981;
  int c3_1 = 1811;
  int c3_2 = 1712;

  auto first = uut.EntityComponentMutable<TestComponent1>(entity);
  auto second = uut.EntityComponentMutable<TestComponent2>(entity);
  auto third = uut.EntityComponentMutable<TestComponent3>(entity);

  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_TRUE(third);

  first.Item1() = c1_1;
  first.Item2() = c1_2;
  second.Item1() = c2_1;
  second.Item2() = c2_2;
  third.Item1() = c3_1;
  third.Item2() = c3_2;
  uut.Update();

  EXPECT_EQ(c1_1, uut.EntityComponent<TestComponent1>(entity).Item1());
  EXPECT_EQ(c1_2, uut.EntityComponent<TestComponent1>(entity).Item2());
  EXPECT_EQ(c2_1, uut.EntityComponent<TestComponent2>(entity).Item1());
  EXPECT_EQ(c2_2, uut.EntityComponent<TestComponent2>(entity).Item2());
  EXPECT_EQ(c3_1, uut.EntityComponent<TestComponent3>(entity).Item1());
  EXPECT_EQ(c3_2, uut.EntityComponent<TestComponent3>(entity).Item2());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueryExistingEntities)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[0]);
  uut.AddComponent<TestComponent3>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[2]);
  uut.Update();

  gazebo::ecs::EntityQuery query;
  query.AddComponent<TestComponent2>();
  query.AddComponent<TestComponent3>();
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
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));

  gazebo::ecs::EntityQuery query;
  query.AddComponent<TestComponent2>();
  query.AddComponent<TestComponent3>();
  auto queryAdd = uut.AddQuery(std::move(query));

  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[0]);
  uut.AddComponent<TestComponent3>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[2]);

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
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[0]);
  uut.AddComponent<TestComponent2>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[0]);
  uut.AddComponent<TestComponent3>(entities[1]);
  uut.AddComponent<TestComponent3>(entities[2]);

  gazebo::ecs::EntityQuery query;
  query.AddComponent<TestComponent2>();
  query.AddComponent<TestComponent3>();

  auto result = uut.AddQuery(std::move(query));
  uut.RemoveQuery(result.first);

  ASSERT_EQ(0, uut.Query(result.first).EntityIds().size());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, TrackComponentChanges)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent3Factory));
  gazebo::ecs::EntityId entity = uut.CreateEntity();

  uut.AddComponent<TestComponent1>(entity);
  // component is created after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent1>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TestComponent1>(entity));

  uut.AddComponent<TestComponent2>(entity);
  // component is created after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TestComponent2>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent1>(entity));

  uut.Update();
  uut.RemoveComponent<TestComponent1>(entity);
  // component is removed after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent1>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TestComponent1>(entity));
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent2>(entity));

  uut.DeleteEntity(entity);
  // component and entity are removed after this update ends
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, uut.IsDifferent<TestComponent2>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TestComponent2>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, QueriesIncludeDeletedEntitiesForOneUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  std::vector<gazebo::ecs::EntityId> entities;
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());
  entities.push_back(uut.CreateEntity());

  uut.AddComponent<TestComponent1>(entities[0]);
  uut.AddComponent<TestComponent1>(entities[1]);
  uut.AddComponent<TestComponent1>(entities[2]);


  gazebo::ecs::EntityQuery query;
  query.AddComponent<TestComponent1>();

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
    EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TestComponent1>(id));
  }

  uut.Update();
  EXPECT_EQ(0, uut.Query(result.first).EntityIds().size());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ModifyThenRemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  uut.AddComponent<TestComponent1>(entity);
  uut.Update();

  ASSERT_TRUE(uut.EntityComponentMutable<TestComponent1>(entity));
  EXPECT_TRUE(uut.RemoveComponent<TestComponent1>(entity));

  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, uut.IsDifferent<TestComponent1>(entity));
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(entity));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, CreateComponentSetInitialValues)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TestComponent1>(entity);
  ASSERT_TRUE(comp);
  comp.Item1() = 12345;

  uut.Update();
  auto constComp = uut.EntityComponent<TestComponent1>(entity);
  ASSERT_TRUE(constComp);
  EXPECT_EQ(12345, comp.Item1());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, MutableComponentHasSetValues)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TestComponent1>(entity);
  ASSERT_TRUE(comp);
  comp.Item1() = 12345;
  uut.Update();

  comp = uut.EntityComponentMutable<TestComponent1>(entity);
  ASSERT_TRUE(comp);
  EXPECT_EQ(12345, comp.Item1());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, ModifyComponentHappensNextUpdate)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId entity = uut.CreateEntity();
  auto comp = uut.AddComponent<TestComponent1>(entity);
  ASSERT_TRUE(comp);
  comp.Item1() = 12345;
  uut.Update();

  comp = uut.EntityComponentMutable<TestComponent1>(entity);
  ASSERT_TRUE(comp);
  comp.Item1() = 6789;

  EXPECT_EQ(gazebo::ecs::WAS_CREATED, uut.IsDifferent<TestComponent1>(entity));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::WAS_MODIFIED, uut.IsDifferent<TestComponent1>(entity));

  auto constComp = uut.EntityComponent<TestComponent1>(entity);
  ASSERT_TRUE(constComp);
  EXPECT_EQ(6789, constComp.Item1());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId id = uut.CreateEntity();

  uut.AddComponent<TestComponent1>(id);
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(id));
  uut.Update();
  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(id));

  uut.RemoveComponent<TestComponent1>(id);
  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(id));
  uut.Update();
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(id));
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, RemoveComponentAccessAnother)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent2Factory));
  gazebo::ecs::EntityId id = uut.CreateEntity();

  auto c1 = uut.AddComponent<TestComponent1>(id);
  auto c2 = uut.AddComponent<TestComponent2>(id);
  ASSERT_TRUE(c1);
  ASSERT_TRUE(c2);

  c1.Item1() = 1234;
  c2.Item1() = 5678;
  c2.Item2() = 9012;

  uut.Update();
  uut.RemoveComponent<TestComponent1>(id);

  auto c2_test = uut.EntityComponent<TestComponent2>(id);
  ASSERT_TRUE(c2_test);
  EXPECT_EQ(5678, c2_test.Item1());
  EXPECT_EQ(9012, c2_test.Item2());

  uut.Update();
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(id));
  c2_test = uut.EntityComponent<TestComponent2>(id);
  ASSERT_TRUE(c2_test);
  EXPECT_EQ(5678, c2_test.Item1());
  EXPECT_EQ(9012, c2_test.Item2());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabase, DeleteEntity)
{
  gazebo::ecs::EntityComponentDatabase uut;
  uut.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::EntityId id = uut.CreateEntity();

  uut.AddComponent<TestComponent1>(id);
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(id));
  uut.Update();
  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(id));

  uut.DeleteEntity(id);
  EXPECT_TRUE(uut.EntityComponent<TestComponent1>(id));
  uut.Update();
  EXPECT_EQ(gazebo::ecs::NO_ENTITY, uut.Entity(id).Id());
  EXPECT_FALSE(uut.EntityComponent<TestComponent1>(id));
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
  db.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  entity.AddComponent<TestComponent1>();
  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_CREATED, entity.IsDifferent<TestComponent1>());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, ReadComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  db.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  auto comp = entity.AddComponent<TestComponent1>();
  ASSERT_TRUE(comp);
  comp.Item1() = 1234;
  db.Update();
  auto readOnlyComp = entity.Component<TestComponent1>();
  ASSERT_TRUE(readOnlyComp);
  EXPECT_EQ(1234, readOnlyComp.Item1());

  db.Update();
  EXPECT_EQ(gazebo::ecs::NO_DIFFERENCE, entity.IsDifferent<TestComponent1>());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, WriteComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  db.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  auto comp = entity.AddComponent<TestComponent1>();
  ASSERT_TRUE(comp);
  comp.Item1() = 1234;
  db.Update();
  auto mutableComp = entity.ComponentMutable<TestComponent1>();
  ASSERT_TRUE(mutableComp);
  mutableComp.Item1() = 4567;

  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_MODIFIED, entity.IsDifferent<TestComponent1>());
  auto readOnlyComp = entity.Component<TestComponent1>();
  EXPECT_EQ(4567, readOnlyComp.Item1());
}

/////////////////////////////////////////////////
TEST(EntityComponentDatabaseEntity, RemoveComponent)
{
  gazebo::ecs::EntityComponentDatabase db;
  db.AddComponentFactory(std::unique_ptr<gazebo::ecs::ComponentFactory>(
        new TestComponent1Factory));
  gazebo::ecs::Entity &entity = db.Entity(db.CreateEntity());
  entity.AddComponent<TestComponent1>();
  db.Update();
  entity.RemoveComponent<TestComponent1>();
  db.Update();
  EXPECT_EQ(gazebo::ecs::WAS_DELETED, entity.IsDifferent<TestComponent1>());
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
