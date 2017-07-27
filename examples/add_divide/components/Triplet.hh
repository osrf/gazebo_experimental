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

#ifndef GAZEBO_EXAMPLES_ADD_DIVIDE_COMPONENTS_TRIPLET_HH_
#define GAZEBO_EXAMPLES_ADD_DIVIDE_COMPONENTS_TRIPLET_HH_

#include "gazebo/server/ComponentManager.hh"

namespace gazebo
{
  namespace components
  {
    /// \brief Three numbers
    struct Triplet
    {
      float first;
      float second;
      float third;
    };

    /// \todo: Create a macro in ComponentManager for this type of operation.
    class TripletRegister
    {
      public: TripletRegister()
              {
                gazebo::server::ComponentManager::Register<
                  gazebo::components::Triplet>(
                      {"triplet", "gazebo::components::Triplet"});
              }
    };

    static TripletRegister tripletRegister;
  }
}

#endif
