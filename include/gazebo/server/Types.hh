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

#ifndef GAZEBO_SERVER_TYPES_HH_
#define GAZEBO_SERVER_TYPES_HH_

namespace gazebo
{
  namespace server
  {
    /// \brief The id for an Entity
    using EntityId = int64_t;

    /// \brief For results which there is no entity
    const EntityId NoEntity = -1;

    /// \brief Id of an EntityQuery
    using EntityQueryId = int64_t;

    /// \brief ID that refers to a component type. A ComponentType number is
    /// designed to uniquely identity a component.
    using ComponentType = int64_t;

    /// \brief This ID uniquely idenfities a particular component instance
    /// within a group of components.
    ///
    /// For example, the component that contains pose information may have
    /// a ComponenType==1, and an instance of a pose may have
    /// a ComponentID==10.
    using ComponentId = int64_t;

    /// \brief Special value returned to say there is no component
    const ComponentType NO_COMPONENT = -1;
  }
}

#endif
