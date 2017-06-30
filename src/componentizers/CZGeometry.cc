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
void CZGeometry::FromSDF(ecs::Manager &_mgr, sdf::Element &_elem,
    const std::unordered_map<sdf::Element*, ecs::EntityId> &_ids)
{
  if (_elem.GetName() == "geometry")
  {
    sdf::ElementPtr parent = _elem.GetParent();
    if (!parent)
    {
      return;
    }
    else if (parent->GetName() == "visual")
    {
      // Populate geometry to entity associated with visual
      ecs::EntityId parentId = _ids.at(parent.get());
      ecs::Entity &parentEntity = _mgr.Entity(parentId);
      auto geom = parentEntity.AddComponent<components::Geometry>();

      // Make sure there is a child with some actual data
      sdf::ElementPtr childElement = _elem.GetFirstElement();
      this->PopulateSimpleGeometry(childElement, geom);
    }
    else if (parent->GetName() == "collision")
    {
      // Geometry component gets added to link
      sdf::ElementPtr superParent = parent->GetParent();
      if (superParent->GetName() != "link")
      {
        return;
      }

      ecs::EntityId superParentId = _ids.at(superParent.get());
      ecs::Entity &superParentEntity = _mgr.Entity(superParentId);
      auto geom = superParentEntity.AddComponent<components::Geometry>();

      // If there are multiple collisions, geometry type will be compound
      if (superParent->GetElement("collision") != parent
          || parent->GetNextElement("collision"))
      {
        ignerr << "TODO support compound geometry\n";
      }
      else
      {
        sdf::ElementPtr childElement = _elem.GetFirstElement();
        this->PopulateSimpleGeometry(childElement, geom);
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
void CZGeometry::PopulateSimpleGeometry(sdf::ElementPtr &_elem,
    components::Geometry &_geom)
{
  if (_elem)
  {
    if (_elem->GetName() == "box")
      this->PopulateBox(_elem, _geom.Shape().Box());
    else if (_elem->GetName() == "sphere")
      this->PopulateSphere(_elem, _geom.Shape().Sphere());
    else if (_elem->GetName() == "cylinder")
      this->PopulateCylinder(_elem, _geom.Shape().Cylinder());
    else
      ignwarn << "Unsupported geometry [" << _elem->GetName() << "]"
        << std::endl;
  }
}

//////////////////////////////////////////////////
void CZGeometry::PopulateBox(sdf::ElementPtr &_elem,
    components::Geometry::BoxGeometry _box)
{
  auto size = _elem->Get<ignition::math::Vector3d>("size");
  _box.X() = size.X();
  _box.Y() = size.Y();
  _box.Z() = size.Z();
}

//////////////////////////////////////////////////
void CZGeometry::PopulateSphere(sdf::ElementPtr &_elem,
    components::Geometry::SphereGeometry _sphere)
{
  _sphere.Radius() = _elem->Get<double>("radius");
}

//////////////////////////////////////////////////
void CZGeometry::PopulateCylinder(sdf::ElementPtr &_elem,
    components::Geometry::CylinderGeometry _cylinder)
{
  _cylinder.Radius() = _elem->Get<double>("radius");
  _cylinder.Length() = _elem->Get<double>("length");
}

//////////////////////////////////////////////////
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::componentizers::CZGeometry,
                                  gazebo::ecs::Componentizer)
