# Gazebo Experimental

This is a prototype for the next version of gazebo using an Entity Component System architecture.

# Building from source

* **Option 1:** use [vcstool and ament](https://github.com/sloretz/gzecs)
* **Option 2:** keep reading

## Get the dependencies
This project uses [CMake](https://cmake.org/).
Most of the important features come from [Ignition Robotics](http://ignitionrobotics.org/).
Build and install the Ignition projects from source in this order:

* [ign-common](https://bitbucket.org/ignitionrobotics/ign-common)
* [ign-tools](https://bitbucket.org/ignitionrobotics/ign-tools) *optional*
* [ign-msgs](https://bitbucket.org/ignitionrobotics/ign-msgs)
* [ign-transport](https://bitbucket.org/ignitionrobotics/ign-transport)
* [ign-gui](https://bitbucket.org/ignitionrobotics/ign-gui)

Other dependencies include

* gflags
    * sudo apt install libgflags-dev

## Get the source code
```
hg clone ssh://hg@bitbucket.org/osrf/gazebo_experimental
```

## Building

```
# Configure the project
cd gazebo_experimental/
mkdir build
cd build/
cmake -DCMAKE_BUILD_TYPE=Debug ..

# build the project
make
# Run tests
make test
# Install the project
make install
```

# Running
This project makes use of plugins to provide most of the features.
The paths to search for these plugins are given using environment variables.
The two important ones are `IGN_GUI_PLUGIN_PATH` and `GAZEBO_PLUGIN_PATH`.

```
# assumes you've done "make install" and are in the build/ folder
export GAZEBO_PLUGIN_PATH=$LD_LIBRARY_PATH
export IGN_GUI_PLUGIN_PATH=src/gui
gazebo -v 4
```

# Uninstalling
**TO DO**

# Style check
**TO DO**

# Code coverage
**TO DO**

# Generate documentation
**TO DO**
