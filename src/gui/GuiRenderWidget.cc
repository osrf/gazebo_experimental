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

#include <ignition/rendering.hh>

#include "GuiRenderWidget.hh"

namespace gzgui = gazebo::gui;
using namespace gzgui;


/////////////////////////////////////////////////
GuiRenderWidget::GuiRenderWidget()
  : Plugin()
{
  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);
  this->setAttribute(Qt::WA_NoSystemBackground, true);

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMouseTracking(true);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

/////////////////////////////////////////////////
GuiRenderWidget::~GuiRenderWidget()
{
}

/////////////////////////////////////////////////
void GuiRenderWidget::showEvent(QShowEvent *_event)
{

  QApplication::flush();

  if (!this->renderWindow)
  {
    std::string engineName = "ogre";
    ignition::rendering::RenderEngine *engine = 
        ignition::rendering::get_engine(engineName);
    if (!engine)
    {
      std::cerr << "Engine '" << engineName << "' is not supported" << std::endl; 
      return;
    }

    // Create sample scene
    ignition::rendering::ScenePtr scene = engine->CreateScene("scene");
    scene->SetAmbientLight(0.3, 0.3, 0.3);
    ignition::rendering::VisualPtr root = scene->RootVisual();
    ignition::rendering::DirectionalLightPtr light0 =
        scene->CreateDirectionalLight();
    light0->SetDirection(-0.5, -0.5, -1);
    light0->SetDiffuseColor(0.5, 0.5, 0.5);
    light0->SetSpecularColor(0.5, 0.5, 0.5);
    root->AddChild(light0);

    ignition::rendering::MaterialPtr green = scene->CreateMaterial();
    green->SetAmbient(0.0, 0.5, 0.0);
    green->SetDiffuse(0.0, 0.7, 0.0);
    green->SetSpecular(0.5, 0.5, 0.5);
    green->SetShininess(50);
    green->SetReflectivity(0);

    ignition::rendering::VisualPtr vis = scene->CreateVisual();
    vis->AddGeometry(scene->CreateBox());
    vis->SetLocalPosition(3, 0, 0);
    vis->SetLocalScale(0.1, 0.1, 0.1);
    vis->SetMaterial(green);
    root->AddChild(vis);

    // create user camera
    this->camera = scene->CreateCamera("user_camera");
    this->camera->SetLocalPosition(0.0, 0.0, 0.0);
    this->camera->SetLocalRotation(0.0, 0.0, 0.0);
//    camera->setImageWidth(800);
//    camera->setImageHeight(800);
    this->camera->SetAspectRatio(1.333);
    this->camera->SetHFOV(M_PI/2.0);
    root->AddChild(camera);

    // create render window
    // windowhandle() is available in qt5 only
    // double ratio = this->windowHandle()->devicePixelRatio();
    double ratio = 1.0;
    std::string winHandle = std::to_string(
				static_cast<uint64_t>(this->winId()));
    this->renderWindow = camera->CreateRenderWindow(winHandle);
//    ignition::rendering::RenderWindow renderWindow = 
//      ignition::rendering::RenderWindowPtr(new RenderWindow());
    this->renderWindow->SetWidth(this->width());
    this->renderWindow->SetHeight(this->height());
    this->renderWindow->SetRatio(ratio);

    // render once to create the window.
   // scene->PreRender();
//    camera->Render();
    this->camera->Update();

    QWidget::showEvent(_event);
    this->setFocus();
  }
}


/////////////////////////////////////////////////
QPaintEngine *GuiRenderWidget::paintEngine() const
{
  return nullptr;
}

/////////////////////////////////////////////////
void GuiRenderWidget::paintEvent(QPaintEvent *_e)
{
  if (this->renderWindow && this->camera)
    this->camera->Update();

  _e->accept();
}


/////////////////////////////////////////////////
void GuiRenderWidget::resizeEvent(QResizeEvent *_e)
{
 if (!this->renderWindow)
    return;
  this->renderWindow->OnResize(_e->size().width(), _e->size().height());
}
// Register this plugin
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::gui::GuiRenderWidget,
                                  ignition::gui::Plugin);
