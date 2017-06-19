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


struct TestComponent1Storage
{
  int item1;
  int item2;
};


gazebo::ecs::ComponentType gTestComponent1Type = gazebo::ecs::NO_COMPONENT;


class TestComponent1 : public gazebo::ecs::ComponentAPI
{
  protected: TestComponent1Storage *dataPtr = nullptr;

  public: TestComponent1()
  {
  }

  public: TestComponent1(TestComponent1Storage *_stor)
  {
    this->dataPtr = _stor;
  }

  public: virtual ~TestComponent1()
  {
  }

  /// \returns true if the component api is fully functional
  public: virtual operator bool() const override
  {
    return dataPtr;
  };

  /// \brief Return the name of the component type
  public: virtual const char *ComponentName() const override
  {
    return "TestComponent1";
  }

  /// \brief Return a unique id for this component
  public: virtual gazebo::ecs::ComponentType ComponentType() const override
  {
    return gTestComponent1Type;
  }

  /// \brief Initializes a static or global variable with a unique id
  /// \remarks this should only allow the type to be set once
  public: virtual void ComponentType(gazebo::ecs::ComponentType _type) override
  {
    gTestComponent1Type = _type;
  }
};


class TestComponent1Factory : public gazebo::ecs::ComponentFactoryHelper<
                              TestComponent1, TestComponent1Storage>
{
};

#endif
