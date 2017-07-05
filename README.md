# Gazebo Experimental

This is a prototype for the next version of gazebo using an Entity Component System architecture.

# Building from source

* **Option 1:** use [vcstool and ament](https://github.com/sloretz/gzecs)
* **Option 2:** keep reading

## Get the dependencies
This project uses [CMake](https://cmake.org/).
Most of the important features come from [Ignition Robotics](http://ignitionrobotics.org/).
Build and install the Ignition projects from source in this order:

* [ign-common](https://bitbucket.org/ignitionrobotics/ign-common) `default` branch
* [ign-gui](https://bitbucket.org/ignitionrobotics/ign-gui) `default` branch
* [ign-msgs](https://bitbucket.org/ignitionrobotics/ign-msgs) `default` branch
* [ign-transport](https://bitbucket.org/ignitionrobotics/ign-transport) `ign-transport3` branch
* [sdformat](https://bitbucket.org/osrf/sdformat) `default` branch

Other dependencies include

* gflags
    * sudo apt install libgflags-dev

## Get the source code
```
hg clone ssh://hg@bitbucket.org/osrf/gazebo_experimental
```

## Building

1. Configure the project

        cd gazebo_experimental/
        mkdir build
        cd build/
        cmake -DCMAKE_BUILD_TYPE=Debug ..

1. Build the project

        make

1. Install the project

        make install

# Running
This project makes use of plugins to provide most of the features.
The paths to search for these plugins are given using environment variables.
The two important ones are `IGN_GUI_PLUGIN_PATH` and `GAZEBO_PLUGIN_PATH`.

1. Set environment variables. Assuming you're in the build/ folder:

        export GAZEBO_PLUGIN_PATH=$LD_LIBRARY_PATH
        export IGN_GUI_PLUGIN_PATH=src/gui

1. If you've done "make install", run Gazebo:

        gazebo -v 4

1. If you haven't installed, from the build/ folder you can run:

        ./src/gazebo

# Running demo

With Gazebo running, you can run the demo from the build folder:

    GAZEBO_PLUGIN_PATH=examples/dummy_demo/systems/ ./examples/dummy_demo/dummy_demo

# Tests

Testing is done using Google Test. Tests are built by default. After building,
to run all tests:

    make test

To run one specific test from the build folder:

    ./test/unit_tests/UNIT_Manager_TEST

# Uninstalling
**TO DO**

# Style check
**TO DO**

# Code coverage
**TO DO**

# Generate documentation
**TO DO**
