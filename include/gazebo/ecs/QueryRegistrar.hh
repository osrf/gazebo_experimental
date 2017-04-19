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

#ifndef GAZEBO_ECS_QUERYREGISTRAR_HH_
#define GAZEBO_ECS_QUERYREGISTRAR_HH_

#include <memory>
#include <utility>

#include "gazebo/ecs/EntityQuery.hh"

namespace gazebo
{
  namespace ecs
  {
    /// \brief forward declaration
    class Manager;

    // Todo cb has a lot of arguments. Maybe have one argument with everything?
    /// \brief typedef for long function pointer
    typedef void (*QueryCallbackPtr)(const EntityQuery &_q, Manager &_m,
        double _dt);

    /// \brief typedef for long registration type
    typedef std::pair<EntityQuery, QueryCallbackPtr> QueryRegistration;

    /// \brief forward declaration
    class QueryRegistrarPrivate;

    /// \brief Used by an ecs::System to register callbacks for query results
    class QueryRegistrar
    {
      public: QueryRegistrar();

      public: ~QueryRegistrar();

      /// \brief adds a query and a callback to call the results
      /// \param[in] _q the query object
      /// \param[in] _cb pointer to callback function
      public: void Register(const EntityQuery &_q, QueryCallbackPtr _cb);

      /// \brief Return the registered callbacks
      public: std::vector<QueryRegistration> Registrations() const;

      /// \brief private implementation
      private: std::shared_ptr<QueryRegistrarPrivate> dataPtr;
    };
  }
}

#endif
