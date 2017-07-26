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
#include "EmptyNested.api.hh"
#include "EnumType.api.hh"
#include "RepeatedMessage.api.hh"
#include "NestedMessage.api.hh"
#include "NestedOneof.api.hh"
#include "OneofMessage.api.hh"
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
  gazebo::components::test::SimpleTypes comp;

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

  auto uutDoubleField = comp.DoubleField();
  auto uutFloatField = comp.FloatField();
  auto uutInt32Field = comp.Int32Field();
  auto uutInt64Field = comp.Int64Field();
  auto uutUint32Field = comp.Uint32Field();
  auto uutUint64Field = comp.Uint64Field();
  auto uutSint32Field = comp.Sint32Field();
  auto uutSint64Field = comp.Sint64Field();
  auto uutFixed32Field = comp.Fixed32Field();
  auto uutFixed64Field = comp.Fixed64Field();
  auto uutSfixed32Field = comp.Sfixed32Field();
  auto uutSfixed64Field = comp.Sfixed64Field();
  auto uutBoolField = comp.BoolField();
  auto uutStringField = comp.StringField();
  auto uutBytesField = comp.BytesField();

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
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, SimpleTypesModifyable)
{
  gazebo::components::test::SimpleTypes comp;

  comp.DoubleField() = 5.0;
  comp.FloatField() = 4.0f;
  comp.Int32Field() = -42;
  comp.Int64Field() = -84;
  comp.Uint32Field() = 42;
  comp.Uint64Field() = 84;
  comp.Sint32Field() = -123;
  comp.Sint64Field() = -456;
  comp.Fixed32Field() = 789;
  comp.Fixed64Field() = 987;
  comp.Sfixed32Field() = -543;
  comp.Sfixed64Field() = -643;
  comp.BoolField() = true;
  comp.StringField() = "Hello world";
  comp.BytesField() = "100001111011";

  EXPECT_EQ(5.0, comp.DoubleField());
  EXPECT_EQ(4.0f, comp.FloatField());
  EXPECT_EQ(-42, comp.Int32Field());
  EXPECT_EQ(-84, comp.Int64Field());
  EXPECT_EQ(42, comp.Uint32Field());
  EXPECT_EQ(84, comp.Uint64Field());
  EXPECT_EQ(-123, comp.Sint32Field());
  EXPECT_EQ(-456, comp.Sint64Field());
  EXPECT_EQ(789, comp.Fixed32Field());
  EXPECT_EQ(987, comp.Fixed64Field());
  EXPECT_EQ(-543, comp.Sfixed32Field());
  EXPECT_EQ(-643, comp.Sfixed64Field());
  EXPECT_EQ(true, comp.BoolField());
  EXPECT_EQ(std::string("Hello world"), comp.StringField());
  EXPECT_EQ(std::string("100001111011"), comp.BytesField());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, MathTypesAreSubstituted)
{
  gazebo::components::test::SubstitutedTypes comp;

  ignition::math::Vector3d VectorField;
  ignition::math::Quaterniond QuaternionField;
  ignition::math::Pose3d PoseField;
  ignition::math::Matrix3d MatrixField;

  auto uutVectorField = comp.Vector();
  auto uutQuaternionField = comp.Quaternion();
  auto uutPoseField = comp.Pose();
  auto uutMatrixField = comp.Transform();

  EXPECT_TRUE(TypesAreSame(VectorField, uutVectorField));
  EXPECT_TRUE(TypesAreSame(QuaternionField, uutQuaternionField));
  EXPECT_TRUE(TypesAreSame(PoseField, uutPoseField));
  EXPECT_TRUE(TypesAreSame(MatrixField, uutMatrixField));
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, MathTypesModifyable)
{
  gazebo::components::test::SubstitutedTypes comp;

  comp.Vector().X(5);
  comp.Quaternion().Y(6);
  comp.Pose().Pos().Z(7);
  comp.Pose().Rot().W(8);

  EXPECT_DOUBLE_EQ(5, comp.Vector().X());
  EXPECT_DOUBLE_EQ(6, comp.Quaternion().Y());
  EXPECT_DOUBLE_EQ(7, comp.Pose().Pos().Z());
  EXPECT_DOUBLE_EQ(8, comp.Pose().Rot().W());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, EnumTypesModifyable)
{
  gazebo::components::test::EnumType comp;

  // default is always zero
  EXPECT_EQ(gazebo::components::test::EnumType::TURTLE, comp.SomeEnum());
  comp.SomeEnum() = gazebo::components::test::EnumType::COW;
  EXPECT_EQ(gazebo::components::test::EnumType::COW, comp.SomeEnum());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, OperatorBool)
{
  gazebo::components::test::EnumType comp1;
  comp1.ComponentType(42);
  gazebo::ecs::NullComponent comp2;

  EXPECT_TRUE(comp1);
  EXPECT_FALSE(comp2);

  // cleanup
  comp1.ComponentType(gazebo::ecs::NO_COMPONENT);
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, DefaultValues)
{
  gazebo::components::test::DefaultValues comp;

  EXPECT_EQ(26, comp.SomeInt());
  EXPECT_EQ(std::string("Hello World!"), comp.SomeString());
  EXPECT_EQ(std::string("1234abc"), comp.SomeBytes());
  // default enum is always zero
  EXPECT_FALSE(comp.SomeBool());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, NestedMessages)
{
  gazebo::components::test::NestedMessage comp;

  // Check imported message works correclty
  comp.Imported().SomeInt() = 35;

  // First level nested message
  comp.FirstLevelMessage().SomeFloat() = 26.7f;
  comp.FirstLevelMessage().SomeInt() = 7;

  // Second level nested message
  comp.DoubleInlined().SecondLevelMessage().SomeFloat() = 8111.1f;
  comp.DoubleInlined().SecondLevelMessage().SomeInt() = -12345;

  // Third level nested message
  comp.DoubleInlined().ThirdLevelMessage().DeepFloat() = -7711.0f;
  comp.DoubleInlined().ThirdLevelMessage().DeepString() = "Hello World!";

  EXPECT_EQ(35, comp.Imported().SomeInt());
  EXPECT_EQ(26.7f, comp.FirstLevelMessage().SomeFloat());
  EXPECT_EQ(7, comp.FirstLevelMessage().SomeInt());
  EXPECT_FLOAT_EQ(8111.1f, comp.DoubleInlined().SecondLevelMessage().SomeFloat());
  EXPECT_EQ(-12345, comp.DoubleInlined().SecondLevelMessage().SomeInt());
  EXPECT_FLOAT_EQ(-7711.0f, comp.DoubleInlined().ThirdLevelMessage().DeepFloat());
  EXPECT_EQ(std::string("Hello World!"), comp.DoubleInlined().ThirdLevelMessage().DeepString());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, RepeatedFields)
{
  gazebo::components::test::RepeatedMessage comp;

  // Check imported message works correclty
  comp.Imported().Resize(2);
  ASSERT_EQ(2, comp.Imported().Size());
  comp.Imported()[0].SomeInt() = 35;
  comp.Imported()[1].SomeInt() = 53;

  comp.InlinedOptional().SomeFloat().Resize(2);
  ASSERT_EQ(2, comp.InlinedOptional().SomeFloat().Size());
  comp.InlinedOptional().SomeFloat()[0] = 5.0f;
  comp.InlinedOptional().SomeFloat()[1] = 26.7f;

  comp.InlinedRepeated().Resize(2);
  ASSERT_EQ(2, comp.InlinedRepeated().Size());
  comp.InlinedRepeated()[0].SomeFloat().Resize(2);
  ASSERT_EQ(2, comp.InlinedRepeated()[0].SomeFloat().Size());
  comp.InlinedRepeated()[1].SomeFloat().Resize(2);
  ASSERT_EQ(2, comp.InlinedRepeated()[1].SomeFloat().Size());
  comp.InlinedRepeated()[0].SomeFloat()[0] = 19.9f;
  comp.InlinedRepeated()[0].SomeFloat()[1] = 0.9f;
  comp.InlinedRepeated()[1].SomeFloat()[0] = 26.0f;
  comp.InlinedRepeated()[1].SomeFloat()[1] = 20.17f;

  EXPECT_EQ(35, comp.Imported()[0].SomeInt());
  EXPECT_EQ(53, comp.Imported()[1].SomeInt());
  EXPECT_FLOAT_EQ(5.0f, comp.InlinedOptional().SomeFloat()[0]);
  EXPECT_FLOAT_EQ(26.7f, comp.InlinedOptional().SomeFloat()[1]);
  EXPECT_FLOAT_EQ(19.9f, comp.InlinedRepeated()[0].SomeFloat()[0]);
  EXPECT_FLOAT_EQ(0.9f, comp.InlinedRepeated()[0].SomeFloat()[1]);
  EXPECT_FLOAT_EQ(26.0f, comp.InlinedRepeated()[1].SomeFloat()[0]);
  EXPECT_FLOAT_EQ(20.17f, comp.InlinedRepeated()[1].SomeFloat()[1]);
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, OneofMessage)
{
  gazebo::components::test::OneofMessage comp;

  // Initially no member is set
  EXPECT_FALSE(comp.SomeUnion().HasImported());
  EXPECT_FALSE(comp.SomeUnion().HasInlined());
  EXPECT_FALSE(comp.SomeUnion().HasSubstituted());
  EXPECT_FALSE(comp.SomeOtherUnion().HasSomeInt());
  EXPECT_FALSE(comp.SomeOtherUnion().HasSomeFloat());

  // default values are set initially
  EXPECT_EQ(26, comp.SomeUnion().Imported().SomeInt());
  EXPECT_EQ(std::string("Hello World!"), comp.SomeUnion().Imported().SomeString());
  EXPECT_EQ(std::string("1234abc"), comp.SomeUnion().Imported().SomeBytes());
  EXPECT_FALSE(comp.SomeUnion().Imported().SomeBool());

  //Accessing a member causes it to be set
  comp.SomeUnion().Imported();
  EXPECT_TRUE(comp.SomeUnion().HasImported());
  comp.SomeUnion().Substituted();
  EXPECT_TRUE(comp.SomeUnion().HasSubstituted());
  comp.SomeUnion().Inlined();
  EXPECT_TRUE(comp.SomeUnion().HasInlined());
  comp.SomeOtherUnion().SomeInt();
  EXPECT_TRUE(comp.SomeOtherUnion().HasSomeInt());
  comp.SomeOtherUnion().SomeFloat();
  EXPECT_TRUE(comp.SomeOtherUnion().HasSomeFloat());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, OneofDeepCopy)
{
  gazebo::components::test::OneofMessage comp1;
  comp1.ComponentType(56);

  comp1.SomeOtherUnion().SomeFloat() = -54.6f;

  gazebo::components::test::OneofMessage comp2;
  comp2.DeepCopy(comp1);

  EXPECT_FLOAT_EQ(-54.6f, comp1.SomeOtherUnion().SomeFloat());
  comp1.SomeOtherUnion().SomeFloat() = -29.5f;
  EXPECT_FLOAT_EQ(-54.6f, comp2.SomeOtherUnion().SomeFloat());

  // Cleanup
  comp1.ComponentType(gazebo::ecs::NO_COMPONENT);
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, DefaultValuesMove)
{
  gazebo::components::test::DefaultValues comp1;
  comp1.ComponentType(56);

  comp1.SomeInt() = 62;

  gazebo::components::test::DefaultValues comp2;
  comp2.Move(comp1);

  EXPECT_EQ(26, comp1.SomeInt());
  EXPECT_EQ(62, comp2.SomeInt());

  // Cleanup
  comp1.ComponentType(gazebo::ecs::NO_COMPONENT);
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, NestedOneof)
{
  gazebo::components::test::NestedOneof comp;

  // First level oneof
  comp.Inlined().SomeUnion().SomeInt() = 1234;
  EXPECT_TRUE(comp.Inlined().SomeUnion().HasSomeInt());
  EXPECT_EQ(1234, comp.Inlined().SomeUnion().SomeInt());

  comp.Inlined().SomeUnion().SomeFloat() = -762.4f;
  EXPECT_TRUE(comp.Inlined().SomeUnion().HasSomeFloat());
  EXPECT_FLOAT_EQ(-762.4f, comp.Inlined().SomeUnion().SomeFloat());

  // Oneof containing nested message containing oneof
  comp.SomeUnion().OneofInlined().SomeUnion().SomeInt() = 1234;
  EXPECT_TRUE(comp.SomeUnion().OneofInlined().SomeUnion().HasSomeInt());
  EXPECT_EQ(1234, comp.SomeUnion().OneofInlined().SomeUnion().SomeInt());

  comp.SomeUnion().OneofInlined().SomeUnion().SomeFloat() = -762.4f;
  EXPECT_TRUE(comp.SomeUnion().OneofInlined().SomeUnion().HasSomeFloat());
  EXPECT_FLOAT_EQ(-762.4f, comp.SomeUnion().OneofInlined().SomeUnion().SomeFloat());
}

/////////////////////////////////////////////////
TEST(PIMPLCPP, EmptyNested)
{
  gazebo::components::test::EmptyNested comp;

  comp.Msg().SomeInt() = 747576;

  EXPECT_EQ(747576, comp.Msg().SomeInt());
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
