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
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include <gflags/gflags.h>

#include <ignition/common/Console.hh>
#include <ignition/common/PluginLoader.hh>
#include <ignition/common/SystemPaths.hh>
#include <ignition/math/Rand.hh>
#include "gazebo/server/ComponentFactory.hh"
#include "gazebo/server/Manager.hh"
#include <sdf/sdf.hh>

#ifndef Q_MOC_RUN
  #include <ignition/gui/Iface.hh>
#endif

#include "gazebo/Config.hh"


// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");
DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");
DEFINE_string(file, "", "");
DEFINE_string(f, "empty.world", "");

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
  << "  -f [ --file ] FILE            SDF file to load on start." << std::endl
  << std::endl;
}

//////////////////////////////////////////////////
void Version()
{
  std::cout << GAZEBO_VERSION_HEADER << std::endl;
}

//////////////////////////////////////////////////
static bool VerbosityValidator(const char * /*_flagname*/, int _value)
{
  return _value >= 0 && _value <= 4;
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
    // Get the SDF file, if specified
    std::string filename = FLAGS_f;
    bool fileSet = false;
    gflags::GetCommandLineFlagInfo("file", &info);
    if (!info.is_default)
    {
      fileSet = true;
      filename = FLAGS_file;
    }

    gflags::GetCommandLineFlagInfo("f", &info);
    if (!info.is_default && fileSet)
    {
      ignerr << "Only one world file may be used at a time. "
        << "The first world file will be used." << std::endl;
    }

    // Set verbosity level
    // This also applies to all upstream libraries using
    // ignition::common::Console
    ignition::common::Console::SetVerbosity(FLAGS_verbose);

    gazebo::server::Manager manager;

    if (!manager.Load(filename))
    {
      ignerr << "Failed to load Gazebo." << std::endl;
      return -1;
    }

    // Run Gazebo server in a separate thread.
    manager.Run();

    // Initialize the GUI
    ignition::gui::initApp();

    // \todo: Load startup plugins and configuration files here before creating
    // the window
    // ignition::gui::loadPlugin("gazeboGuiDisplayImage");
    // ignition::gui::loadPlugin("gazeboGuiDiagnostics");

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

    // Stop the server
    manager.Stop();
    igndbg << "Waiting for the server thread" << std::endl;
  }

  igndbg << "Shutting down" << std::endl;
  return 0;
}
