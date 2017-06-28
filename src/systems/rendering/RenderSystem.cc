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

#include "gazebo/ecs/Entity.hh"
#include "gazebo/ecs/Manager.hh"
#include "gazebo/ecs/EntityQuery.hh"

#include "RenderSystem.hh"

using namespace gazebo;
using namespace systems;

/////////////////////////////////////////////////
RenderSystem::RenderSystem()
{
}

/////////////////////////////////////////////////
RenderSystem::~RenderSystem()
{
}

/////////////////////////////////////////////////
void RenderSystem::Init(ecs::QueryRegistrar &_registrar)
{
  ecs::EntityQuery query;

  _registrar.Register(query, std::bind(&RenderSystem::Update, this,
        std::placeholders::_1));

  std::string topic = "/rendering/image";
  this->pub =
      this->node.Advertise<ignition::msgs::Image>(topic);
  if (!this->pub)
    std::cerr << "Error advertising topic [" << topic << "]" << std::endl;
}


/////////////////////////////////////////////////
void RenderSystem::Update(const ecs::EntityQuery &_result)
{
  if (!this->camera)
  {
    this->camera = this->CreateCamera("ogre");
    this->image = std::make_shared<ignition::rendering::Image>(
        this->camera->CreateImage());
  }

  // the render engine should only be responsible for updating the scene tree.
  // we shouldn't be throttling the update rate based on sim time
  // because objects can move when simulation is paused!
  // This is just for demo only
  auto &mgr = this->Manager();
  auto const &currentSimTime = mgr.SimulationTime();
  double updateRate = 1000.0;
  if ((currentSimTime - this->prevUpdateTime).Double() < 1.0/updateRate)
    return;
  this->prevUpdateTime = currentSimTime;
  // for demo only
  // move the camera slowly in x, y, and z
  static double offset = 0.0;
  this->camera->SetLocalPosition(offset, offset, offset);
  offset += 0.0001;

  // rendering system should not be controlling the framerate
  // Most of the logic below should be moved to sensors and gui

  // throttle framerate
  double framerate = 60.0;
  auto const &currentTime = ignition::common::Time::SystemTime();
  if ((currentTime - prevRenderTime).Double() < 1.0/framerate)
    return;
  this->prevRenderTime = currentTime;

  this->camera->Capture(*this->image);
  unsigned char *data = this->image->Data<unsigned char>();

  // publish
  ignition::msgs::Image img;
  img.set_width(this->camera->ImageWidth());
  img.set_height(this->camera->ImageHeight());
  img.set_step(this->camera->ImageWidth() *
      this->camera->ImageDepth());
  img.set_pixel_format(3);
  img.set_data(data, this->camera->ImageWidth() *
      this->camera->ImageHeight() *
      this->camera->ImageDepth());
  this->pub.Publish(img);


}

/////////////////////////////////////////////////
ignition::rendering::CameraPtr RenderSystem::CreateCamera(
    const std::string &_engineName)
{
  // create and populate scene
  ignition::rendering::RenderEngine *engine =
      ignition::rendering::get_engine(_engineName);
  if (!engine)
  {
    std::cout << "Engine '" << _engineName
              << "' is not supported" << std::endl;
    return ignition::rendering::CameraPtr();
  }
  ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
  this->BuildScene(scene);

  // return camera sensor
  ignition::rendering::SensorPtr sensor = scene->SensorByName("camera");
  return std::dynamic_pointer_cast<ignition::rendering::Camera>(sensor);
}

/////////////////////////////////////////////////
void RenderSystem::BuildScene(ignition::rendering::ScenePtr _scene)
{
  // for demo only
  // add entities to the scene

  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  ignition::rendering::VisualPtr root = _scene->RootVisual();

  // create point light
  ignition::rendering::DirectionalLightPtr light0 =
      _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create point light
  ignition::rendering::PointLightPtr light2 = _scene->CreatePointLight();
  light2->SetDiffuseColor(0.5, 0.5, 0.5);
  light2->SetSpecularColor(0.5, 0.5, 0.5);
  light2->SetLocalPosition(3, 5, 5);
  root->AddChild(light2);

  // create green material
  ignition::rendering::MaterialPtr green = _scene->CreateMaterial();
  green->SetAmbient(0.0, 0.5, 0.0);
  green->SetDiffuse(0.0, 0.7, 0.0);
  green->SetSpecular(0.5, 0.5, 0.5);
  green->SetShininess(50);
  green->SetReflectivity(0);

  // create center visual
  ignition::rendering::VisualPtr center = _scene->CreateVisual();
  center->AddGeometry(_scene->CreateSphere());
  center->SetLocalPosition(3, 0, 0);
  center->SetLocalScale(0.1, 0.1, 0.1);
  center->SetMaterial(green);
  root->AddChild(center);

  // create red material
  ignition::rendering::MaterialPtr red = _scene->CreateMaterial();
  red->SetAmbient(0.5, 0.0, 0.0);
  red->SetDiffuse(1.0, 0.0, 0.0);
  red->SetSpecular(0.5, 0.5, 0.5);
  red->SetShininess(50);
  red->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr sphere = _scene->CreateVisual();
  sphere->AddGeometry(_scene->CreateSphere());
  sphere->SetOrigin(0.0, -0.5, 0.0);
  sphere->SetLocalPosition(3, 0, 0);
  sphere->SetLocalRotation(0, 0, 0);
  sphere->SetLocalScale(1, 2.5, 1);
  sphere->SetMaterial(red);
  root->AddChild(sphere);

  // create blue material
  ignition::rendering::MaterialPtr blue = _scene->CreateMaterial();
  blue->SetAmbient(0.0, 0.0, 0.3);
  blue->SetDiffuse(0.0, 0.0, 0.8);
  blue->SetSpecular(0.5, 0.5, 0.5);
  blue->SetShininess(50);
  blue->SetReflectivity(0);

  // create box visual
  ignition::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(M_PI / 4, 0, M_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(blue);
  root->AddChild(box);

  // create white material
  ignition::rendering::MaterialPtr white = _scene->CreateMaterial();
  white->SetAmbient(0.5, 0.5, 0.5);
  white->SetDiffuse(0.8, 0.8, 0.8);
  white->SetReceiveShadows(true);
  white->SetReflectivity(0);

  // create sphere visual
  ignition::rendering::VisualPtr plane = _scene->CreateVisual();
  plane->AddGeometry(_scene->CreatePlane());
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(white);
  root->AddChild(plane);

  // create camera
  ignition::rendering::CameraPtr camera = _scene->CreateCamera("camera");
  camera->SetLocalPosition(0.0, 0.0, 0.0);
  camera->SetLocalRotation(0.0, 0.0, 0.0);
  camera->SetImageWidth(800);
  camera->SetImageHeight(600);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(1.333);
  camera->SetHFOV(M_PI / 2);
  root->AddChild(camera);
}

IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::systems::RenderSystem,
                                  gazebo::ecs::System)
