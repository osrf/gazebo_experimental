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

#ifndef GAZEBO_ECS_COMPONENTAPI_HH_
#define GAZEBO_ECS_COMPONENTAPI_HH_

#include <memory>
#include <type_traits>

// TODO move this to ComponentFactor.hh

namespace gazebo
{
  namespace ecs
  {
    /// \brief ID to refer to a component type
    typedef std::size_t ComponentType;

    /// \brief Special value returned to say there is no component
    static const ComponentType NO_COMPONENT = -1;


    /// \brief Class that offers an API for a component, but stores no data
    /// \description This class is meant to give PIMPL-like access to a
    ///              ComponentStorage instance. It should have no member
    ///              variables other than a private pointer
    class Component
    {
      /// \brief virtual destructor
      public: virtual ~Component();

      /// \returns true if ComponentType() != NO_COMPONENT
      public: operator bool() const;

      /// \brief Return the name of the component type
      public: virtual const char *ComponentName() const = 0;

      /// \brief Return a unique id for this component
      public: virtual ecs::ComponentType ComponentType() const = 0;

      /// \brief Initializes a static or global variable with a unique id
      /// \remarks this should only allow the type to be set once
      public: virtual void ComponentType(ecs::ComponentType _type) = 0;

      /// \brief TODO apis for introspection and serialization/deserialization
    };

    /// \brief Used to indicate a component does not exist
    class NullComponent : public Component
    {
      /// \brief virtual destructor
      public: virtual ~NullComponent();

      /// \brief Returns "gazebo::ecs::NullComponent"
      public: virtual const char *ComponentName() const override;

      /// \returns NO_COMPONENT
      public: virtual ecs::ComponentType ComponentType() const override;

      /// \brief Does nothing
      public: virtual void ComponentType(ecs::ComponentType _type) override;
    };
  }
}
#endif
