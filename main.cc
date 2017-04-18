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
#include <map>

#include <gflags/gflags.h>

#include "gazebo/Config.hh"

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");

// Additional flags will go here

void Help()
{
  std::cerr << "gazebo -- Run the Gazebo server and GUI.\n" << std::endl;
  std::cerr << "`gazebo` [options] <world_file>\n" << std::endl;
  // std::cerr << "Gazebo server runs simulation and handles commandline "
  //   << "options, starts a Master, runs World update and sensor generation "
  //   << "loops. This also starts the Gazebo GUI client in a separate "
  //   << "process.\n" << std::endl;

  std::cerr << "Options:" << std::endl
  // << "  -d [ --record ]               Record simulation data." << std::endl
  // << "  -e [ --physics ] arg          Specify a physics engine "
  // << "  -f [ --file ] arg             Load an SDF file on start." << std::endl
  // << "  -g [ --gui ]                  Run only the GUI." << std::endl
  << "  -h [ --help ]                 Print help message." << std::endl
  // << "  -i [ --iters ] arg            Run simulation for the specified "
  // <<                                  "number of iterations." << std::endl
  // <<                                  "(xxx|xxx|xxx)." << std::endl
  // << "  -l [ --replay ] arg           Replay a log file." << std::endl
  // << "  --minimal_comms               Reduce the amount of generated TCP/IP"
  // << "  -o [ --profile ] arg          Physics profile name." << std::endl
  // << "  -p [ --plugin ] arg           Load a plugin." << std::endl
  // << "  -r [ --run ]                  Run physics simulation on start."
  // <<                                  std::endl
  // << "  --record_encoding arg (=zlib) Compression encoding format for log "
  // <<                                  "data " << std::endl
  // << "                                (zlib|bz2|txt)." << std::endl
  // << "  -s [ --server ]               Run only the server, a.k.a. headless."
  // <<                                  std::endl
  // << "  --seed arg                    Start with the given random number seed."
  // <<                                  std::endl
  // <<                                  " traffic." << std::endl
  // << "  -v [ --verbose ] level (=X)   Adjust the level of console output."
  // <<                                  std::endl
  << "  --version                     Print version information." << std::endl
  << std::endl;
}

void Version()
{
  std::cerr << GAZEBO_VERSION_HEADER << std::endl;
}

int main(int _argc, char **_argv)
{
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Parse out the help flag in such a way that the full help text
  // is suppressed: if --help* or -h is specified, override the default
  // help behavior and turn on --helpmatch, to only shows help for the
  // current executable (instead of showing a huge list of gflags built-ins).
  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);

  bool showHelp = FLAGS_h;
  bool showVersion = false;
  for (auto const &flag : flags)
  {
    showHelp = showHelp || (flag.name.find("help") != std::string::npos &&
                            flag.current_value == "true");
    showVersion = showVersion ||
                  (flag.name.find("version") != std::string::npos &&
                      flag.current_value == "true");
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
  if (showVersion)
  {
    gflags::SetCommandLineOption("version", "false");
    Version();
  }

  return 0;
}
