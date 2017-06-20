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
#include <iostream>
#include <ignition/common/PluginMacros.hh>

#include "dummy_rendering/Object.hh"

#include "gazebo/components/Geometry.api.hh"
#include "gazebo/components/Material.api.hh"
#include "gazebo/components/Pose.api.hh"
#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/EntityQuery.hh"

#include "DummyRendering.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
void DummyRendering::Init(ecs::QueryRegistrar &_registrar)
{
  ecs::EntityQuery query;

  _registrar.Register(query, std::bind(&DummyRendering::Update, this,
        std::placeholders::_1));

  std::string topic = "/rendering/image";
  this->pub = this->node.Advertise<ignition::msgs::Image>(topic);
  if (!this->pub)
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
}

/////////////////////////////////////////////////
void DummyRendering::Update(const ecs::EntityQuery &_result)
{
  auto &mgr = this->Manager();
  auto const &currentTime = mgr.SimulationTime();
  if (currentTime < this->nextRenderTime)
  {
    // Too early to publish
    return;
  }
  double framerate = 30.0;
  this->nextRenderTime += ignition::common::Time(1.0 / framerate);

  for (auto const &entityId : _result.EntityIds())
  {
    auto &entity = mgr.Entity(entityId);
    auto difference_material = entity.IsDifferent<components::Material>();
    auto difference_geometry = entity.IsDifferent<components::Geometry>();
    auto difference_position = entity.IsDifferent<components::Pose>();
    dummy_rendering::Object *obj = this->scene.GetById(entityId);

    const bool doDelete = ecs::WAS_DELETED == difference_material ||
      ecs::WAS_DELETED == difference_geometry ||
      ecs::WAS_DELETED == difference_position;

    const bool doCreate = ecs::WAS_CREATED == difference_material ||
      ecs::WAS_CREATED == difference_geometry ||
      ecs::WAS_CREATED == difference_position;

    const bool doModify = ecs::WAS_MODIFIED == difference_material ||
      ecs::WAS_MODIFIED == difference_geometry;

    if (obj && doDelete)
    {
      this->RemoveObjectFromScene(entity);
    }
    else if (obj == nullptr || doCreate)
    {
      this->AddObjectToScene(entity);
    }
    else if (doModify)
    {
      this->RemoveObjectFromScene(entity);
      this->AddObjectToScene(entity);
    }

    if (ecs::WAS_MODIFIED == difference_position)
    {
      this->UpdatePosition(entity);
    }
  }

  this->PublishImages();
}

/////////////////////////////////////////////////
void DummyRendering::AddObjectToScene(ecs::Entity &_entity)
{
  // Ogre seems to have a scene manager that objects need to be added to
  auto material = _entity.Component<components::Material>();
  auto geometry = _entity.Component<components::Geometry>();

  if (geometry.Shape().HasSphere() && material.Appearance().HasColor())
  {
    auto pose = _entity.Component<components::Pose>();
    dummy_rendering::Object obj;
    obj.scene_x = pose.Origin().Pos().X();
    obj.scene_y = pose.Origin().Pos().Y();
    obj.scene_z = pose.Origin().Pos().Z();
    obj.radius = geometry.Shape().Sphere().Radius();
    obj.red = material.Appearance().Color().Red() * 255;
    obj.green = material.Appearance().Color().Green() * 255;
    obj.blue = material.Appearance().Color().Blue() * 255;
    this->scene.AddObject(_entity.Id(), obj);
  }
}

/////////////////////////////////////////////////
void DummyRendering::RemoveObjectFromScene(ecs::Entity &_entity)
{
  this->scene.RemoveObject(_entity.Id());
}

/////////////////////////////////////////////////
void DummyRendering::UpdatePosition(ecs::Entity &_entity)
{
  auto pose = _entity.Component<components::Pose>();
  if (pose)
  {
    dummy_rendering::Object *obj = this->scene.GetById(_entity.Id());
    if (obj)
    {
      obj->scene_x = pose.Origin().Pos().X();
      obj->scene_y = pose.Origin().Pos().Y();
      obj->scene_z = pose.Origin().Pos().Z();
    }
  }
  else
    std::cerr << "[rendering] Pose is null!" << std::endl;
}

/////////////////////////////////////////////////
void DummyRendering::PublishImages()
{
  // Render
  std::vector<uint8_t> imageData = this->scene.GetImage(1000,1000);

  // publish
  ignition::msgs::Image img;
  img.set_width(1000);
  img.set_height(1000);
  img.set_step(3000);
  img.set_pixel_format(3);
  auto data = img.mutable_data();
  data->reserve(imageData.size());
  for (uint8_t channel : imageData)
  {
    data->push_back(channel);
  }
  std::cout << "[rendering] publishing " << std::endl;
  this->pub.Publish(img);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::DummyRendering,
                                  gazebo::ecs::System)
