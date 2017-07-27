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

#ifndef GAZEBO_COMPONENTS_POSE_HH_
#define GAZEBO_COMPONENTS_POSE_HH_

#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include "gazebo/server/ComponentFactory.hh"

namespace gazebo
{
  namespace components
  {
    /// \brief Create a Pose3d component from SDF
    ignition::math::Pose3d Pose3dFromSDF(sdf::ElementPtr _elem)
    {
      if (_elem->GetName() == "pose")
      {
        return _elem->Get<ignition::math::Pose3d>();
      }
      else
      {
        ignerr << "Attempting to read an ignition::math::pose3d "
          << "value from an SDF element of type["
          << _elem->GetName() << "].\n";
      }
      return ignition::math::Pose3d();
    }

    /// \todo: Create a macro in ComponentFactory for this type of operation.
    class Pose3dRegister
    {
      public: Pose3dRegister()
              {
                gazebo::server::ComponentFactory::Register<
                  ignition::math::Pose3d>({"pose", "ignition::math::Pose3d"},
                      std::bind(&Pose3dFromSDF, std::placeholders::_1));
              }
    };
    static Pose3dRegister poseRegister;
  }
}

#endif
