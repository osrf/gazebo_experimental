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

#ifndef GAZEBO_DUMMYRENDERING_SCENE_HH_
#define GAZEBO_DUMMYRENDERING_SCENE_HH_

#include <algorithm>
#include <cstdint>
#include <map>
#include <vector>
#include <utility>

#include "dummy_rendering/Object.hh"

namespace dummy_rendering
{
  class Scene
  {
    public: Scene()
      {
      }

    public: ~Scene()
      {
      }

    /// \brief adds an object to be rendered
    public: void AddObject(int _id, Object _obj)
      {
        this->objects[_id] = _obj;
      }

    /// \brief return an object by it's id
    public: Object *GetById(int _id)
      {
        Object *obj = nullptr;
        auto iter = this->objects.find(_id);
        if (iter != this->objects.end())
        {
          obj = &(iter->second);
        }
        return obj;
      }

    /// \brief removes object so it's no longer renderable
    public: void RemoveObject(int _id)
      {
        this->objects.erase(_id);
      }

    /// \brief returns image data 3 bytes per pixel
    public: std::vector<uint8_t> GetImage(int width, int height)
      {
        // background color black
        uint8_t initial_color = 0;
        // Make an orthogonal image
        std::vector<uint8_t> buffer(3 * width * height, initial_color);

        std::vector<std::pair<int, float> > zOrder;
        for (auto &kv : this->objects)
        {
          zOrder.push_back(std::make_pair(kv.first, kv.second.scene_z));
        }

        // Get in order of back to front
        std::sort(zOrder.begin(), zOrder.end(),
            [] (std::pair<int, float> a, std::pair<int, float> b) -> bool {
              return a.second < b.second;
            });

        for (auto pair : zOrder)
        {
          this->objects[pair.first].Render(buffer, width, height);
        }
        return buffer;
      }

    /// \brief objects to render
    public: std::map<int, Object> objects;
  };
}
#endif
