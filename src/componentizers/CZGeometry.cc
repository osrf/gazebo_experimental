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
#include <ignition/common/Console.hh>
#include <ignition/common/PluginMacros.hh>
#include <gazebo/components/Geometry.api.hh>
#include "CZGeometry.hh"

namespace gzcompz = gazebo::componentizers;
using namespace gazebo;
using namespace gzcompz;

//////////////////////////////////////////////////
CZGeometry::~CZGeometry()
{
}

//////////////////////////////////////////////////
void CZGeometry::Init()
{
}

//////////////////////////////////////////////////
void CZGeometry::FromSDF(std::unique_ptr<ecs::DataHandle> _handle,
    sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "geometry")
  {
    // Look for a parent element. The geometry component will be grouped with
    // the other components created for it.
    sdf::ElementPtr parent = _elem.GetParent();
    if (parent &&
        (parent->GetName() == "visual" || parent->GetName() == "collision"))
    {
      ecs::EntityId parentId = _ids.at(parent.get());
      ecs::Entity &parentEntity = _handle->Entity(parentId);

      // Make sure there is a child with some actual data
      sdf::ElementPtr childElement = _elem.GetFirstElement();
      if (childElement)
      {
        if (childElement->GetName() == "box")
          this->AttachBox(childElement, parentEntity);
        else if (childElement->GetName() == "sphere")
          this->AttachSphere(childElement, parentEntity);
        else if (childElement->GetName() == "cylinder")
          this->AttachCylinder(childElement, parentEntity);
        else
          ignwarn << "Unsupported geometry [" << childElement->GetName() << "]"
            << std::endl;
      }
    }
    else
    {
      if (parent)
        ignwarn << "unknown parent tag <" << parent->GetName() << ">"
          << std::endl;
      else
        ignwarn << "geometry tag with no parent" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void CZGeometry::AttachBox(sdf::ElementPtr &_elem, ecs::Entity &_entity)
{
  auto geom = _entity.AddComponent<components::Geometry>();
  auto size = _elem->Get<ignition::math::Vector3d>("size");
  geom.Shape().Box().X() = size.X();
  geom.Shape().Box().Y() = size.Y();
  geom.Shape().Box().Z() = size.Z();
  igndbg << "Added box to " << _entity.Id() << std::endl;
}

//////////////////////////////////////////////////
void CZGeometry::AttachSphere(sdf::ElementPtr &_elem, ecs::Entity &_entity)
{
  auto geom = _entity.AddComponent<components::Geometry>();
  geom.Shape().Sphere().Radius() = _elem->Get<double>("radius");
  igndbg << "Added Sphere to " << _entity.Id() << std::endl;
}

//////////////////////////////////////////////////
void CZGeometry::AttachCylinder(sdf::ElementPtr &_elem, ecs::Entity &_entity)
{
  auto geom = _entity.AddComponent<components::Geometry>();
  geom.Shape().Cylinder().Radius() = _elem->Get<double>("radius");
  geom.Shape().Cylinder().Length() = _elem->Get<double>("length");
  igndbg << "Added Cylinder to " << _entity.Id() << std::endl;
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZGeometry,
                                  gazebo::ecs::Componentizer)
