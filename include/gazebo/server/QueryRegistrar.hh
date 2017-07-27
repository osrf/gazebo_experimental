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

#ifndef GAZEBO_SERVER_QUERYREGISTRAR_HH_
#define GAZEBO_SERVER_QUERYREGISTRAR_HH_

#include <functional>
#include <memory>
#include <utility>

#include "gazebo/server/EntityQuery.hh"

namespace gazebo
{
  namespace server
  {
    class Manager;

    /// \brief typedef for query callbacks
    typedef std::function<void (const Manager *_mgr,
        const EntityQuery &_q)> QueryCallback;

    /// \brief typedef for long registration type
    typedef std::pair<EntityQuery, QueryCallback> QueryRegistration;

    /// \brief forward declaration
    class EntityQueryRegistrarPrivate;

    /// \brief Used by a server::System to register callbacks for query results
    class EntityQueryRegistrar
    {
      /// \brief Constructor
      public: EntityQueryRegistrar();

      /// \brief Destructor
      public: ~EntityQueryRegistrar();

      /// \brief Add a query and a callback to call the results
      /// \param[in] _q The query object
      /// \param[in] _cb Callback function
      public: void Register(EntityQuery &&_q, QueryCallback _cb);

      /// \brief Return the registered callbacks
      public: std::vector<QueryRegistration> &Registrations() const;

      /// \brief private implementation
      private: std::unique_ptr<EntityQueryRegistrarPrivate> dataPtr;
    };
  }
}

#endif
