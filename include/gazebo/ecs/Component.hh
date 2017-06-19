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

#ifndef GAZEBO_ECS_COMPONENTAPI_HH_
#define GAZEBO_ECS_COMPONENTAPI_HH_

#include <memory>
#include <type_traits>

// TODO move this to ComponentFactor.hh

namespace gazebo
{
  namespace ecs
  {
    /// \brief ID to refer to a component type
    typedef std::size_t ComponentType;

    /// \brief Special value returned to say there is no component
    static const ComponentType NO_COMPONENT = -1;


    /// \brief Class that offers an API for a component, but stores no data
    /// \description This class is meant to give PIMPL-like access to a
    ///              ComponentStorage instance. It should have no member
    ///              variables other than a private pointer
    class ComponentAPI
    {
      /// \brief virtual destructor
      public: virtual ~ComponentAPI()
      {
      }

      /// \returns true if the component api is fully functional
      public: virtual operator bool() const = 0;

      /// \brief Return the name of the component type
      public: virtual const char *ComponentName() const = 0;

      /// \brief Return a unique id for this component
      public: virtual ecs::ComponentType ComponentType() const = 0;

      /// \brief Initializes a static or global variable with a unique id
      /// \remarks this should only allow the type to be set once
      public: virtual void ComponentType(ecs::ComponentType _type) = 0;

      /// \brief TODO apis for introspection and serialization/deserialization
    };


    /// \brief Abstract Base class for a component factory
    /// \description This has important information the database needs to
    ///              manage component storage and placement. Components are
    ///              divided into two parts: Storage and API. The Storage is
    ///              where all data in the component lives. The database needs
    ///              information about the size and how to copy the storage so
    ///              that it can position them in memory to minimize cache
    ///              misses. The API is a PIMPL'd class that's given to the
    ///              user. It's job is to be an API that won't break ABI if the
    ///              component storage gets additional fields. The database
    ///              needs to know how to construct it with access to the
    ///              storage instance.
    class ComponentFactory
    {
      /// \brief virtual destructor
      public: virtual ~ComponentFactory()
      {
      }

      /// \brief Returns the unique name of the component
      public: const char *ComponentName() const
      {
        // Make dummy storage and API to get component name
        char *dummyStorage = new char[this->StorageSize()];
        char *dummyAPI = new char[this->APISize()];
        void *vStor = static_cast<void*>(dummyStorage);
        void *vAPI = static_cast<void*>(dummyAPI);
        this->ConstructStorage(vStor);
        this->ConstructAPI(vAPI, vStor);

        const char *name = static_cast<ComponentAPI*>(vAPI)->ComponentName();

        this->DestructAPI(vAPI);
        this->DestructStorage(vStor);
        delete [] dummyAPI;
        delete [] dummyStorage;
        return name;
      }

      /// \brief Sets the component type used to identify the API
      public: const char *ComponentType(ecs::ComponentType _type)
      {
        // Make dummy storage and API to set component type
        char *dummyStorage = new char[this->StorageSize()];
        char *dummyAPI = new char[this->APISize()];
        void *vStor = static_cast<void*>(dummyStorage);
        void *vAPI = static_cast<void*>(dummyAPI);
        this->ConstructStorage(vStor);
        this->ConstructAPI(vAPI, vStor);

        static_cast<ComponentAPI*>(vAPI)->ComponentType(_type);

        this->DestructAPI(vAPI);
        this->DestructStorage(vStor);
        delete [] dummyAPI;
        delete [] dummyStorage;
      }

      /// \brief Gets the component type used to identify the API
      public: ecs::ComponentType ComponentType()
      {
        // Make dummy storage and API to set component type
        char *dummyStorage = new char[this->StorageSize()];
        char *dummyAPI = new char[this->APISize()];
        void *vStor = static_cast<void*>(dummyStorage);
        void *vAPI = static_cast<void*>(dummyAPI);
        this->ConstructStorage(vStor);
        this->ConstructAPI(vAPI, vStor);

        ecs::ComponentType type =
          static_cast<ComponentAPI*>(vAPI)->ComponentType();

        this->DestructAPI(vAPI);
        this->DestructStorage(vStor);
        delete [] dummyAPI;
        delete [] dummyStorage;
        return type;
      }

      /// \brief Returns the size in bytes needed to store this component
      public: virtual std::size_t StorageSize() const  = 0;

      /// \brief Returns the size in bytes needed to store API class
      public: virtual std::size_t APISize() const = 0;

      /// \brief Instantiates API class in place with storage
      public: virtual void ConstructAPI(void *_location,
                  void *_storage) const = 0;

      /// \brief Destructs API class in place
      public: virtual void DestructAPI(void *_location) const = 0;

      /// \brief Instantiates Storage class at the given address
      public: virtual void ConstructStorage(void *_location) const = 0;

      /// \brief Destructs Storage class at the given address
      public: virtual void DestructStorage(void *_location) const = 0;

      /// \brief Shallow copy storage from one location to another
      public: virtual void ShallowCopyStorage(void *_from, void *_to) const = 0;

      /// \brief Deep copy storage from one location to another
      public: virtual void DeepCopyStorage(void *_from, void *_to) const = 0;
    };


    /// \brief template to help a derived class of ComponentFactory
    /// \remarks assumes API has a constructor which takes STORAGE*
    /// \remarks assumes STORAGE has a constructor with no arguments
    /// \remarks assumes STORAGE copy constructor does a deep copy
    template <typename API, typename STORAGE>
    class ComponentFactoryHelper : public ComponentFactory
    {
      static_assert(std::is_base_of<ComponentAPI, API>::value,
          "API class must inherit from Component API");

      static_assert(std::is_copy_constructible<STORAGE>::value,
          "STORAGE class must perform a deep copy in its copy constructor");

      static_assert(std::is_constructible<API, STORAGE*>::value,
          "API class must have contructor with argument (STORAGE*)");

      public: virtual ~ComponentFactoryHelper()
      {
        // Nothing to do
      }

      public: virtual std::size_t StorageSize() const override
      {
        return sizeof(STORAGE);
      }

      public: virtual std::size_t APISize() const override
      {
        return sizeof(API);
      }

      public: virtual void ConstructAPI(void *_location, void *_storage) const
              override
      {
        // Cast void to STORAGE*
        STORAGE *storage = static_cast<STORAGE*>(_storage);
        // placement new operator so this doesn't allocate memory
        new (_location) API(storage);
      }

      public: virtual void DestructAPI(void *_location) const override
      {
        // explicit call to destructor without freeing memory
        static_cast<API*>(_location)->~API();
      }

      public: virtual void ConstructStorage(void *_location) const override
      {
        // placement new operator so this doesn't allocate memory
        new(_location) STORAGE();
      }

      public: virtual void DestructStorage(void *_location) const override
      {
        // explicit call to destructor without freeing memory
        static_cast<STORAGE*>(_location)->~STORAGE();
      }

      public: virtual void ShallowCopyStorage(void *_from, void *_to) const
              override
      {
        // Copy each byte from one location to another
        // The compiler should unroll this loop completely
        for (int i = 0; i < sizeof(STORAGE); ++i)
        {
          static_cast<char*>(_to)[i] =
            static_cast<char const*>(_from)[i];
        }
      }

      public: virtual void DeepCopyStorage(void *_from, void *_to) const
              override
      {
        // Copy component using its copy constructor and placement new.
        const STORAGE *src = static_cast<const STORAGE *>(_from);
        new(_to) STORAGE(static_cast<const STORAGE &>(*src));
      }
    };
  }
}
#endif
