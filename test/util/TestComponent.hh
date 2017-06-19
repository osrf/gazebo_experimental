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

#ifndef GAZEBO_TEST_UTIL_TEST_COMPONENT_HH_
#define GAZEBO_TEST_UTIL_TEST_COMPONENT_HH_

#include <gazebo/ecs/Component.hh>


struct TestComponentStorage
{
  int item1;
  int item2;
};


#define GZTEST_TEST_COMPONENT(num) \
gazebo::ecs::ComponentType gTestComponent##num##Type = num; \
 \
class TestComponent##num : public gazebo::ecs::ComponentAPI \
{ \
  protected: TestComponentStorage *dataPtr = nullptr; \
 \
  public: TestComponent##num() \
  { \
  } \
 \
  public: TestComponent##num(TestComponentStorage *_stor) \
  { \
    this->dataPtr = _stor; \
  } \
 \
  public: virtual ~TestComponent##num() \
  { \
  } \
 \
  public: virtual operator bool() const override \
  { \
    return dataPtr; \
  }; \
 \
  public: virtual const char *ComponentName() const override \
  { \
    return "TestComponent" #num; \
  } \
 \
  public: virtual gazebo::ecs::ComponentType ComponentType() const override \
  { \
    return gTestComponent##num##Type; \
  } \
 \
  public: virtual void ComponentType( \
              gazebo::ecs::ComponentType _type) override \
  { \
    gTestComponent##num##Type = _type; \
  } \
}; \
 \
class TestComponent##num##Factory : public gazebo::ecs::ComponentFactoryHelper< \
                              TestComponent##num, TestComponentStorage> \
{ \
};

GZTEST_TEST_COMPONENT(1);
GZTEST_TEST_COMPONENT(2);
GZTEST_TEST_COMPONENT(3);

#endif
