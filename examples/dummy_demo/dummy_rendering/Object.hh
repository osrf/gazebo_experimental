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

#ifndef GAZEBO_DUMMYRENDERING_OBJECT_HH_
#define GAZEBO_DUMMYRENDERING_OBJECT_HH_

#include <cstdint>
#include <vector>

namespace dummy_rendering
{
  class Object
  {
    public: Object()
      {
      }

    public: ~Object()
      {
      }

    /// \brief draw this object into an image
    public: void Render(std::vector<uint8_t> &_image, int _width, int _height)
      {
        const float scene_width = 10.0;
        const float scene_height = 10.0;
        const float scene_depth = 10.0;

        // convert from right handed coordinate system to screen coordinates
        // screen x increases from left to right
        // screen y increases from top to bottom
        int screen_x = (_width / (-1.0 * scene_width)) * this->scene_y + _width / 2.0;
        int screen_y = (_height / (-1.0 * scene_height)) * this->scene_x + _height / 2.0;

        float x_radius = _width / scene_width * this->radius;
        float y_radius = _height / scene_height * this->radius;

        int min_x = screen_x - x_radius;
        if (min_x < 0)
          min_x = 0;
        int max_x = screen_x + x_radius;
        if (max_x > _width)
          max_x = _width;

        int min_y = screen_y - y_radius;
        if (min_y < 0)
          min_y = 0;
        int max_y = screen_y + y_radius;
        if (max_y > _height)
          max_y = _height;

        for (int px = min_x; px < max_x; ++px)
        {
          for (int py = min_y; py < max_y; ++py)
          {
            // It's a sphere? Draw an orthographic square
            int pixel_red = (_width * py + px) * 3;
            int pixel_green = pixel_red + 1;
            int pixel_blue = pixel_red + 2;
            _image[pixel_red] = this->red;
            _image[pixel_green] = this->green;
            _image[pixel_blue] = this->blue;
          }
        }
      }

    /// \brief where this is in scene coordinates (center)
    public: float scene_x;
    public: float scene_y;
    public: float scene_z;
    /// \brief how big is this in the scene
    public: float radius;

    /// \brief color of object
    public: uint8_t red;
    public: uint8_t green;
    public: uint8_t blue;
  };
}
#endif
