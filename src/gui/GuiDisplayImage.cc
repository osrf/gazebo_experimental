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
#include <ignition/common/Image.hh>

#include "GuiDisplayImage.hh"

namespace gzgui = gazebo::gui;
using namespace gzgui;


/////////////////////////////////////////////////
GuiDisplayImage::GuiDisplayImage()
  : pixmap(128, 128), Plugin()
{
  // Show a blank image at first
  this->pixmap.fill();
  this->label = new QLabel;
  this->label->setPixmap(this->pixmap);

  // Create the layout to hold the button
  auto layout = new QHBoxLayout;
  layout->addWidget(this->label);

  // Use the layout
  this->setLayout(layout);

  // Subscribe to get images
  // TODO How to tell this gui plugin which topic to subscribe to?
  std::string topic = "/rendering/image";
  if (!node.Subscribe(topic, &GuiDisplayImage::OnImageReceived, this))
  {
    std::cerr << "Unable to subscribe to topic" << std::endl;
  }
}

/////////////////////////////////////////////////
std::string GuiDisplayImage::Title()
{
  std::string title = "Image: ";
  auto topics = this->node.SubscribedTopics();
  if (topics.size())
  {
    title += topics[0];
  }
  return title;
}

/////////////////////////////////////////////////
GuiDisplayImage::~GuiDisplayImage()
{
}

/////////////////////////////////////////////////
Q_SLOT void GuiDisplayImage::OnImageChanged()
{
  std::lock_guard<std::mutex> lock(this->mtx);
  switch(this->img.pixel_format())
  {
    case ignition::common::Image::RGB_INT8:
      this->UpdateFromRgbInt8();
      break;
    default:
      std::cerr << "Unsupported image type: " << img.pixel_format()
        << std::endl;
  }
}

/////////////////////////////////////////////////
void GuiDisplayImage::OnImageReceived(const ignition::msgs::Image &_img)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  this->img = _img;

  // Signal to GUI (main) thread that the image changed
  QMetaObject::invokeMethod(this, "OnImageChanged");
}

/////////////////////////////////////////////////
void GuiDisplayImage::UpdateFromRgbInt8()
{
  QImage image(this->img.width(), this->img.height(), QImage::Format_RGB888);

  auto const &data = this->img.data();
  for (int y_pixel = 0; y_pixel < this->img.height(); ++y_pixel)
  {
    for (int x_pixel = 0; x_pixel < this->img.width(); ++x_pixel)
    {
      int idx = x_pixel + y_pixel * this->img.width();
      unsigned char red = data[3 * idx];
      unsigned char green = data[3 * idx + 1];
      unsigned char blue = data[3 * idx + 2];
      image.setPixel(x_pixel, y_pixel, qRgb(red, green, blue));
    }
  }
  this->pixmap.convertFromImage(image);
  this->label->setPixmap(this->pixmap);
}

// Register this plugin
IGN_COMMON_REGISTER_SINGLE_PLUGIN(gazebo::gui::GuiDisplayImage,
                                  ignition::gui::Plugin);
