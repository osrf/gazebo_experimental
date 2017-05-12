# Gazebo Experimental

This is a prototype for the next version of gazebo using an Entity Component System architecture.

# Building from source

* **Option 1:** use [vcstool and ament](https://github.com/sloretz/gzecs)
* **Option 2:** keep reading

## Get the dependencies
This project relies on [Ignition Robotics](http://ignitionrobotics.org/).
Build and install them from source in this order:

* [ign-common]()
* [ign-tools]() *optional*
* [ign-msgs]()
* [ign-transport]()
* [ign-gui]()

Other dependencies include

* gflags
    * sudo apt install libgflags-dev

## Building

This project uses [CMake](https://cmake.org/).

```
# Configure the project
cd gazebo_experimental/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Debug ..

# build the project
make
```

## Running Tests
```
make test
```

# Running
This project makes use of plugins to provide most of the features.
The paths to search for these plugins are given using environment variables.
The two important ones are `IGN_GUI_PLUGIN_PATH` and `GAZEBO_PLUGIN_PATH`.

For example, this will run a demo with a GUI window.

```
# assumes you've done "make install" and are in the build/ folder
GAZEBO_PLUGIN_PATH=$LD_LIBRARY_PATH examples/dummy_demo/dummy_demo &
IGN_GUI_PLUGIN_PATH=src/gui gazebo
```

# Installing
**TO DO**

# Uninstalling
**TO DO**

# Style check
**TO DO**

# Code coverage
**TO DO**

# Generate documentation
**TO DO**
