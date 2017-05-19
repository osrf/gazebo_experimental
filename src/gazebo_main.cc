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

#include <atomic>
#include <functional>
#include <iostream>
#include <thread>

#include <gflags/gflags.h>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/math/Rand.hh>
#include "gazebo/components/Inertial.hh"
#include "gazebo/components/Geometry.hh"
#include "gazebo/components/PhysicsProperties.hh"
#include "gazebo/components/WorldPose.hh"
#include "gazebo/components/WorldVelocity.hh"
#include "gazebo/ecs/ComponentFactory.hh"
#include "gazebo/ecs/Manager.hh"

#ifndef Q_MOC_RUN
  #include <ignition/gui/Iface.hh>
#endif

#include "gazebo/Config.hh"

namespace gzecs = gazebo::ecs;

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");
DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");

//////////////////////////////////////////////////
void Help()
{
  std::cout
  << "gazebo -- Run the Gazebo server and GUI." << std::endl
  << std::endl
  << "`gazebo` [options] <world_file>" << std::endl
  << std::endl
  << std::endl
  << "Options:" << std::endl
  << "  -h [ --help ]                 Print help message." << std::endl
  << "  --version                     Print version information." << std::endl
  << "  -v [--verbose] arg            Adjust the level of console output (0~4)."
  << std::endl
  << std::endl;
}

//////////////////////////////////////////////////
void Version()
{
  std::cout << GAZEBO_VERSION_HEADER << std::endl;
}

//////////////////////////////////////////////////
void Verbose()
{
  // This also applies to all upstream libraries using ignition::common::Console
  ignition::common::Console::SetVerbosity(FLAGS_verbose);
}

//////////////////////////////////////////////////
static bool VerbosityValidator(const char */*_flagname*/, int _value)
{
  return _value >= 0 && _value <= 4;
}


//////////////////////////////////////////////////
bool LoadSystems(gzecs::Manager &_mgr, std::vector<std::string> _libs)
{
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");

  for (auto const &libName : _libs)
  {
    std::string pathToLibrary = sp.FindSharedLibrary(libName);
    std::string pluginName = pluginLoader.LoadLibrary(pathToLibrary);
    if (pluginName.size())
    {
      std::unique_ptr<gzecs::System> sys;
      sys = pluginLoader.Instantiate<gzecs::System>(pluginName);
      if (!_mgr.LoadSystem(pluginName, std::move(sys)))
      {
        ignerr << "Failed to load " << pluginName << " from " << libName
          << std::endl;
        return false;
      }
      else
      {
        igndbg << "Loaded plugin " << pluginName << " from " << libName
          << std::endl;
      }
    }
    else
    {
      ignerr << "Failed to load library " << libName << std::endl;
      return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool LoadComponentizers(gzecs::Manager &_mgr, std::vector<std::string> _libs)
{
  ignition::common::PluginLoader pluginLoader;
  ignition::common::SystemPaths sp;
  sp.SetPluginPathEnv("GAZEBO_PLUGIN_PATH");

  for (auto const &libName : _libs)
  {
    std::string pathToLibrary = sp.FindSharedLibrary(libName);
    std::string pluginName = pluginLoader.LoadLibrary(pathToLibrary);
    if (pluginName.size())
    {
      std::unique_ptr<gzecs::Componentizer> cz;
      cz = pluginLoader.Instantiate<gzecs::Componentizer>(pluginName);
      if (!_mgr.LoadComponentizer(std::move(cz)))
      {
        ignerr << "Failed to load " << pluginName << " from " << libName
          << std::endl;
        return false;
      }
      else
      {
        igndbg << "Loaded plugin " << pluginName << " from " << libName
          << std::endl;
      }
    }
    else
    {
      ignerr << "Failed to load library " << libName << std::endl;
      return false;
    }
  }
  return true;
}

//////////////////////////////////////////////////
std::string PlaceholderLoadWorld()
{
  return std::string(
      "<?xml version='1.0'?>"
      "<sdf version='1.6'>"
      " <world name='default'>"
      "   <model name='some_model'>"
      "     <link name='some_link'>"
      "       <pose>0 0 0 0 0 0</pose>"
      "       <collision name='some_collision'>"
      "         <geometry>"
      "           <box>"
      "             <size>1 2 3</size>"
      "           </box>"
      "         </geometry>"
      "       </collision>"
      "       <collision name='some_other_collision'>"
      "         <pose>1 0 0 0 0 0</pose>"
      "         <geometry>"
      "           <box>"
      "             <size>1 2 3</size>"
      "           </box>"
      "         </geometry>"
      "       </collision>"
      "       <visual name='some_visual'>"
      "         <geometry>"
      "           <box>"
      "             <size>1 2 3</size>"
      "           </box>"
      "         </geometry>"
      "       </visual>"
      "       <visual name='some_other_visual'>"
      "         <pose>1 0 0 0 0 0</pose>"
      "         <geometry>"
      "           <box>"
      "             <size>1 2 3</size>"
      "           </box>"
      "         </geometry>"
      "       </visual>"
      "       <inertial>"
      "         <mass>5.0</mass>"
      "         <inertia>"
      "           <ixx>1.0</ixx>"
      "           <ixy>2.0</ixy>"
      "           <ixz>3.0</ixz>"
      "           <iyy>0</iyy>"
      "           <iyz>0</iyz>"
      "           <izz>0</izz>"
      "         </inertia>"
      "       </inertial>"
      "     </link>"
      "     <link name='some_link_2'>"
      "       <pose>1.5 0 1 0 0 0</pose>"
      "       <collision name='some_collision_2'>"
      "         <geometry>"
      "           <sphere>"
      "             <radius>0.5</radius>"
      "           </sphere>"
      "         </geometry>"
      "       </collision>"
      "       <visual name='some_visual'>"
      "         <geometry>"
      "           <sphere>"
      "             <radius>0.5</radius>"
      "           </sphere>"
      "         </geometry>"
      "       </visual>"
      "     </link>"
      "     <link name='some_link_3'>"
      "       <pose>3 0 2 0 0 0</pose>"
      "       <collision name='some_collision_3'>"
      "         <geometry>"
      "           <cylinder>"
      "             <radius>0.5</radius>"
      "             <length>1.0</length>"
      "           </cylinder>"
      "         </geometry>"
      "       </collision>"
      "       <visual name='some_visual'>"
      "         <geometry>"
      "           <cylinder>"
      "             <radius>0.5</radius>"
      "             <length>1.0</length>"
      "           </cylinder>"
      "         </geometry>"
      "       </visual>"
      "     </link>"
      "   </model>"
      " </world>"
      "</sdf>"
    );
}

//////////////////////////////////////////////////
void PlaceholderCreateComponents(gzecs::Manager &_mgr)
{
  // Componentizer should register components
  gzecs::ComponentFactory::Register<gazebo::components::Inertial>(
      "gazebo::components::Inertial");
  gzecs::ComponentFactory::Register<gazebo::components::Geometry>(
      "gazebo::components::Geometry");
  gzecs::ComponentFactory::Register<gazebo::components::PhysicsProperties>(
      "gazebo::components::PhysicsProperties");
  gzecs::ComponentFactory::Register<gazebo::components::WorldPose>(
      "gazebo::components::WorldPose");
  gzecs::ComponentFactory::Register<gazebo::components::WorldVelocity>(
      "gazebo::components::WorldVelocity");

  // Placeholder create components. The componentizer should do this
  // Create 25 sphere entities
  for (int i = 0; i < 25; i++)
  {
    // Create the entity
    gzecs::EntityId e = _mgr.CreateEntity();
    gzecs::Entity &entity = _mgr.Entity(e);

    // Inertial component
    auto inertial = entity.AddComponent<gazebo::components::Inertial>();
    if (inertial)
    {
      inertial->mass = ignition::math::Rand::DblUniform(0.1, 5.0);
    }
    else
    {
      std::cerr << "Failed to add inertial component to entity [" << e << "]"
                << std::endl;
    }

    // Geometry component
    auto geom = entity.AddComponent<gazebo::components::Geometry>();
    if (geom)
    {
      geom->type = gazebo::components::Geometry::SPHERE;
      geom->sphere.radius = ignition::math::Rand::DblUniform(0.1, 0.5);
    }
    else
    {
      std::cerr << "Failed to add geom component to entity [" << e << "]"
                << std::endl;
    }

    // World pose
    auto pose = entity.AddComponent<gazebo::components::WorldPose>();
    if (pose)
    {
      pose->position.X(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Y(ignition::math::Rand::DblUniform(-4.0, 4.0));
      pose->position.Z(ignition::math::Rand::DblUniform(-4.0, 4.0));
    }
    else
    {
      std::cerr << "Failed to add world pose component to entity [" << e << "]"
                << std::endl;
    }

    // World velocity
    auto vel = entity.AddComponent<gazebo::components::WorldVelocity>();
    if (vel)
    {
      vel->linear.X(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Y(ignition::math::Rand::DblUniform(-1.0, 1.0));
      vel->linear.Z(ignition::math::Rand::DblUniform(-1.0, 1.0));
    }
    else
    {
      std::cerr << "Failed to add world velocity component to entity ["
                << e << "]" << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void RunECS(gzecs::Manager &_mgr, std::atomic<bool> &stop)
{
  const double realTimeFactor = 1.0;
  while (!stop)
  {
    _mgr.UpdateOnce(realTimeFactor);
  }
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Register validators
  gflags::RegisterFlagValidator(&FLAGS_verbose, &VerbosityValidator);
  gflags::RegisterFlagValidator(&FLAGS_v, &VerbosityValidator);

  // Parse command line
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Hold info as we parse it
  gflags::CommandLineFlagInfo info;

  // Help
  // Parse out the help flag in such a way that the full help text
  // is suppressed: if --help* or -h is specified, override the default
  // help behavior and turn on --helpmatch, to only shows help for the
  // current executable (instead of showing a huge list of gflags built-ins).
  gflags::GetCommandLineFlagInfo("help", &info);
  bool showHelp = FLAGS_h || (info.current_value == "true");

  // Version
  gflags::GetCommandLineFlagInfo("version", &info);
  bool showVersion = (info.current_value == "true");

  // Verbosity
  gflags::GetCommandLineFlagInfo("verbose", &info);
  if (info.is_default)
  {
    gflags::GetCommandLineFlagInfo("v", &info);
    if (!info.is_default)
      FLAGS_verbose = FLAGS_v;
  }

  // If help message is requested, substitute in the override help function.
  if (showHelp)
  {
    gflags::SetCommandLineOption("help", "false");
    gflags::SetCommandLineOption("helpshort", "false");
    gflags::SetCommandLineOption("helpfull", "false");
    gflags::SetCommandLineOption("helpmatch", "");
    Help();
  }
  // If version is requested, override with custom version print function.
  else if (showVersion)
  {
    gflags::SetCommandLineOption("version", "false");
    Version();
  }
  // Run Gazebo
  else
  {
    // Set verbosity level
    Verbose();

    gzecs::Manager manager;

    if (!LoadComponentizers(manager, {
          "gazeboCZName",
          "gazeboCZGeometry",
          "gazeboCZMaterial",
          "gazeboCZPose",
          "gazeboCZCollidable",
          "gazeboCZInertial",
          }))
    {
      return 2;
    }

    // TODO componentizer
    // PlaceholderCreateComponents(manager);

    // Load ECS systems
    if (!LoadSystems(manager, {
        "gazeboPhysicsSystem",
        }))
    {
      return 1;
    }

    // Load the world
    manager.LoadWorld(PlaceholderLoadWorld());

    // Initialize app
    ignition::gui::initApp();

    // Run the ECS in another thread
    std::atomic<bool> stop(false);
    std::thread ecsThread(RunECS, std::ref(manager), std::ref(stop));

    // TODO: load startup plugins and configuration files here before creating
    // the window
    ignition::gui::loadPlugin("gazeboGuiDisplayImage");
    ignition::gui::loadPlugin("gazeboGuiDiagnostics");

    // Create main window
    ignition::gui::createMainWindow();

    // Customize window
    auto win = ignition::gui::mainWindow();
    win->setWindowTitle("Gazebo");

    // Run main window - this blocks until the window is closed or we receive a
    // SIGINT
    ignition::gui::runMainWindow();

    // Cleanup once main window is closed
    ignition::gui::stop();

    // Stop the ECS
    stop = true;
    igndbg << "Waiting for ECS thread" << std::endl;
    ecsThread.join();
  }

  igndbg << "Shutting down" << std::endl;
  return 0;
}
