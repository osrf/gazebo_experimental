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

#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>

#include "gazebo/ecs/Component.hh"

// Generated header files
#include "DefaultValues.api.hh"
#include "EnumType.api.hh"
#include "RepeatedMessage.api.hh"
#include "NestedMessage.api.hh"
#include "SimpleTypes.api.hh"
#include "SubstitutedTypes.api.hh"


// Type comparison
// https://stackoverflow.com/questions/11861610/decltype-comparison
template<typename T, typename U>
typename std::enable_if<std::is_same<T, U>::value, bool>::type
TypesAreSame(T& t, U& u)
{
    // Types are same
    return true;
}
template<typename T, typename U>
typename std::enable_if<!std::is_same<T, U>::value, bool>::type
TypesAreSame(T& t, U& u)
{
    // types are different
    return false;
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, SimpleHaveRightTypes)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentSimpleTypes");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::SimpleTypesFactory", pluginName);

  auto stFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[stFactory->StorageSize()]);
  stFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::SimpleTypes st;
  stFactory->ConstructAPI(
      static_cast<void *>(&st), static_cast<void *>(storage.get()));

  double DoubleField;
  float FloatField;
  int32_t Int32Field;
  int64_t Int64Field;
  uint32_t Uint32Field;
  uint64_t Uint64Field;
  int32_t Sint32Field;
  int64_t Sint64Field;
  uint32_t Fixed32Field;
  uint64_t Fixed64Field;
  int32_t Sfixed32Field;
  int64_t Sfixed64Field;
  bool BoolField;
  std::string StringField;
  std::string BytesField;

  auto uutDoubleField = st.DoubleField();
  auto uutFloatField = st.FloatField();
  auto uutInt32Field = st.Int32Field();
  auto uutInt64Field = st.Int64Field();
  auto uutUint32Field = st.Uint32Field();
  auto uutUint64Field = st.Uint64Field();
  auto uutSint32Field = st.Sint32Field();
  auto uutSint64Field = st.Sint64Field();
  auto uutFixed32Field = st.Fixed32Field();
  auto uutFixed64Field = st.Fixed64Field();
  auto uutSfixed32Field = st.Sfixed32Field();
  auto uutSfixed64Field = st.Sfixed64Field();
  auto uutBoolField = st.BoolField();
  auto uutStringField = st.StringField();
  auto uutBytesField = st.BytesField();

  EXPECT_TRUE(TypesAreSame(DoubleField, uutDoubleField));
  EXPECT_TRUE(TypesAreSame(FloatField, uutFloatField));
  EXPECT_TRUE(TypesAreSame(Int32Field, uutInt32Field));
  EXPECT_TRUE(TypesAreSame(Int64Field, uutInt64Field));
  EXPECT_TRUE(TypesAreSame(Uint32Field, uutUint32Field));
  EXPECT_TRUE(TypesAreSame(Uint64Field, uutUint64Field));
  EXPECT_TRUE(TypesAreSame(Sint32Field, uutSint32Field));
  EXPECT_TRUE(TypesAreSame(Sint64Field, uutSint64Field));
  EXPECT_TRUE(TypesAreSame(Fixed32Field, uutFixed32Field));
  EXPECT_TRUE(TypesAreSame(Fixed64Field, uutFixed64Field));
  EXPECT_TRUE(TypesAreSame(Sfixed32Field, uutSfixed32Field));
  EXPECT_TRUE(TypesAreSame(Sfixed64Field, uutSfixed64Field));
  EXPECT_TRUE(TypesAreSame(BoolField, uutBoolField));
  EXPECT_TRUE(TypesAreSame(StringField, uutStringField));
  EXPECT_TRUE(TypesAreSame(BytesField, uutBytesField));

  stFactory->DestructAPI(static_cast<void *>(&st));
  stFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, SimpleTypesModifyable)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentSimpleTypes");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::SimpleTypesFactory", pluginName);

  auto stFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[stFactory->StorageSize()]);
  stFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::SimpleTypes st;
  stFactory->ConstructAPI(
      static_cast<void *>(&st), static_cast<void *>(storage.get()));

  st.DoubleField() = 5.0;
  st.FloatField() = 4.0f;
  st.Int32Field() = -42;
  st.Int64Field() = -84;
  st.Uint32Field() = 42;
  st.Uint64Field() = 84;
  st.Sint32Field() = -123;
  st.Sint64Field() = -456;
  st.Fixed32Field() = 789;
  st.Fixed64Field() = 987;
  st.Sfixed32Field() = -543;
  st.Sfixed64Field() = -643;
  st.BoolField() = true;
  st.StringField() = "Hello world";
  st.BytesField() = "100001111011";

  EXPECT_EQ(5.0, st.DoubleField());
  EXPECT_EQ(4.0f, st.FloatField());
  EXPECT_EQ(-42, st.Int32Field());
  EXPECT_EQ(-84, st.Int64Field());
  EXPECT_EQ(42, st.Uint32Field());
  EXPECT_EQ(84, st.Uint64Field());
  EXPECT_EQ(-123, st.Sint32Field());
  EXPECT_EQ(-456, st.Sint64Field());
  EXPECT_EQ(789, st.Fixed32Field());
  EXPECT_EQ(987, st.Fixed64Field());
  EXPECT_EQ(-543, st.Sfixed32Field());
  EXPECT_EQ(-643, st.Sfixed64Field());
  EXPECT_EQ(true, st.BoolField());
  EXPECT_EQ(std::string("Hello world"), st.StringField());
  EXPECT_EQ(std::string("100001111011"), st.BytesField());

  stFactory->DestructAPI(static_cast<void *>(&st));
  stFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, MathTypesAreSubstituted)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentSubstitutedTypes");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::SubstitutedTypesFactory", pluginName);

  auto stFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[stFactory->StorageSize()]);
  stFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::SubstitutedTypes st;
  stFactory->ConstructAPI(
      static_cast<void *>(&st), static_cast<void *>(storage.get()));

  ignition::math::Vector3d VectorField;
  ignition::math::Quaterniond QuaternionField;
  ignition::math::Pose3d PoseField;

  auto uutVectorField = st.Vector();
  auto uutQuaternionField = st.Quaternion();
  auto uutPoseField = st.Pose();

  EXPECT_TRUE(TypesAreSame(VectorField, uutVectorField));
  EXPECT_TRUE(TypesAreSame(QuaternionField, uutQuaternionField));
  EXPECT_TRUE(TypesAreSame(PoseField, uutPoseField));

  stFactory->DestructAPI(static_cast<void *>(&st));
  stFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, MathTypesModifyable)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentSubstitutedTypes");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::SubstitutedTypesFactory", pluginName);

  auto stFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[stFactory->StorageSize()]);
  stFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::SubstitutedTypes st;
  stFactory->ConstructAPI(
      static_cast<void *>(&st), static_cast<void *>(storage.get()));

  st.Vector().X(5);
  st.Quaternion().Y(6);
  st.Pose().Pos().Z(7);
  st.Pose().Rot().W(8);

  EXPECT_DOUBLE_EQ(5, st.Vector().X());
  EXPECT_DOUBLE_EQ(6, st.Quaternion().Y());
  EXPECT_DOUBLE_EQ(7, st.Pose().Pos().Z());
  EXPECT_DOUBLE_EQ(8, st.Pose().Rot().W());

  stFactory->DestructAPI(static_cast<void *>(&st));
  stFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, EnumTypesModifyable)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentEnumType");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::EnumTypeFactory", pluginName);

  auto etFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[etFactory->StorageSize()]);
  etFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::EnumType et;
  etFactory->ConstructAPI(
      static_cast<void *>(&et), static_cast<void *>(storage.get()));

  // default is always zero
  EXPECT_EQ(gazebo::components::test::EnumType::TURTLE, et.SomeEnum());
  et.SomeEnum() = gazebo::components::test::EnumType::COW;
  EXPECT_EQ(gazebo::components::test::EnumType::COW, et.SomeEnum());

  etFactory->DestructAPI(static_cast<void *>(&et));
  etFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, DefaultValues)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentDefaultValues");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::DefaultValuesFactory", pluginName);

  auto dvFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[dvFactory->StorageSize()]);
  dvFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::DefaultValues dv;
  dvFactory->ConstructAPI(
      static_cast<void *>(&dv), static_cast<void *>(storage.get()));

  EXPECT_EQ(26, dv.SomeInt());
  EXPECT_EQ(std::string("Hello World!"), dv.SomeString());
  EXPECT_EQ(std::string("1234abc"), dv.SomeBytes());
  // default enum is always zero
  EXPECT_FALSE(dv.SomeBool());

  dvFactory->DestructAPI(static_cast<void *>(&dv));
  dvFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, NestedMessages)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentNestedMessage");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::NestedMessageFactory", pluginName);

  auto nmFactory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[nmFactory->StorageSize()]);
  nmFactory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::NestedMessage nm;
  nmFactory->ConstructAPI(
      static_cast<void *>(&nm), static_cast<void *>(storage.get()));

  // Check imported message works correclty
  nm.Imported().SomeInt() = 35;

  // First level nested message
  nm.FirstLevelMessage().SomeFloat() = 26.7f;
  nm.FirstLevelMessage().SomeInt() = 7;

  // Second level nested message
  nm.DoubleInlined().SecondLevelMessage().SomeFloat() = 8111.1f;
  nm.DoubleInlined().SecondLevelMessage().SomeInt() = -12345;

  // Third level nested message
  nm.DoubleInlined().ThirdLevelMessage().DeepFloat() = -7711.0f;
  nm.DoubleInlined().ThirdLevelMessage().DeepString() = "Hello World!";

  EXPECT_EQ(35, nm.Imported().SomeInt());
  EXPECT_EQ(26.7f, nm.FirstLevelMessage().SomeFloat());
  EXPECT_EQ(7, nm.FirstLevelMessage().SomeInt());
  EXPECT_FLOAT_EQ(8111.1f, nm.DoubleInlined().SecondLevelMessage().SomeFloat());
  EXPECT_EQ(-12345, nm.DoubleInlined().SecondLevelMessage().SomeInt());
  EXPECT_FLOAT_EQ(-7711.0f, nm.DoubleInlined().ThirdLevelMessage().DeepFloat());
  EXPECT_EQ(std::string("Hello World!"), nm.DoubleInlined().ThirdLevelMessage().DeepString());

  nmFactory->DestructAPI(static_cast<void *>(&nm));
  nmFactory->DestructStorage(static_cast<void *>(storage.get()));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, RepeatedFields)
{
  ignition::common::PluginLoader pl;
  ignition::common::SystemPaths sp;
  sp.AddPluginPaths("./");
  std::string pathToLibrary = sp.FindSharedLibrary(
      "gazeboComponentRepeatedMessage");
  ASSERT_FALSE(pathToLibrary.empty());
  std::string pluginName = pl.LoadLibrary(pathToLibrary);
  ASSERT_EQ("::gazebo::components::test::RepeatedMessageFactory", pluginName);

  auto factory = pl.Instantiate<gazebo::ecs::ComponentFactory>(pluginName);
  std::unique_ptr<char> storage(new char[factory->StorageSize()]);
  factory->ConstructStorage(static_cast<void *>(storage.get()));

  gazebo::components::test::RepeatedMessage rm;
  factory->ConstructAPI(
      static_cast<void *>(&rm), static_cast<void *>(storage.get()));

  // Check imported message works correclty
  rm.Imported().resize(2);
  rm.Imported()[0].SomeInt() = 35;
  rm.Imported()[1].SomeInt() = 53;

  rm.InlinedOptional().SomeFloat().resize(2);
  rm.InlinedOptional().SomeFloat()[0] = 5.0f;
  rm.InlinedOptional().SomeFloat()[1] = 26.7f;

  rm.InlinedRepeated().resize(2);
  rm.InlinedRepeated()[0].SomeFloat().resize(2);
  rm.InlinedRepeated()[1].SomeFloat().resize(2);
  rm.InlinedRepeated()[0].SomeFloat()[0] = 19.9f;
  rm.InlinedRepeated()[0].SomeFloat()[1] = 0.9f;
  rm.InlinedRepeated()[1].SomeFloat()[0] = 26.0f;
  rm.InlinedRepeated()[1].SomeFloat()[1] = 20.17f;

  EXPECT_EQ(35, rm.Imported()[0].SomeInt());
  EXPECT_EQ(53, rm.Imported()[1].SomeInt());
  EXPECT_FLOAT_EQ(5.0f, rm.InlinedOptional().SomeFloat()[0]);
  EXPECT_FLOAT_EQ(26.7f, rm.InlinedOptional().SomeFloat()[1]);
  EXPECT_FLOAT_EQ(19.9f, rm.InlinedRepeated()[0].SomeFloat()[0]);
  EXPECT_FLOAT_EQ(0.9f, rm.InlinedRepeated()[0].SomeFloat()[1]);
  EXPECT_FLOAT_EQ(26.0f, rm.InlinedRepeated()[1].SomeFloat()[0]);
  EXPECT_FLOAT_EQ(20.17f, rm.InlinedRepeated()[1].SomeFloat()[1]);

  factory->DestructAPI(static_cast<void *>(&rm));
  factory->DestructStorage(static_cast<void *>(storage.get()));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

