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
#include <sstream>

#include <gflags/gflags.h>

#include "gazebo/Config.hh"

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "--help, Print help message");

// Additional flags will go here

int main(int _argc, char **_argv)
{
  std::stringstream stream;
  stream << "Run the Gazebo server and GUI." << std::endl
         << std::endl
         << "`gazebo` [options]";
  gflags::SetUsageMessage(stream.str());
  gflags::SetVersionString(GAZEBO_VERSION_HEADER);
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Parse out the help flag in such a way that the full help text
  // is suppressed: if --help* or -h is specified, override the default
  // help behavior and turn on --helpmatch, to only shows help for the
  // current executable (instead of showing a huge list of gflags built-ins).
  std::vector<gflags::CommandLineFlagInfo> flags;
  gflags::GetAllFlags(&flags);

  bool showHelp = false;
  for (auto const &flag : flags)
  {
    showHelp = showHelp || (flag.name.find("help") != std::string::npos &&
                            flag.current_value == "true");
  }
  // If help message is requested, substitute help for helpmatch.
  if (showHelp || FLAGS_h)
  {
    gflags::SetCommandLineOption("help", "false");
    gflags::SetCommandLineOption("helpmatch", "main");
  }

  // print help and/or version information
  gflags::HandleCommandLineHelpFlags();

  return 0;
}
