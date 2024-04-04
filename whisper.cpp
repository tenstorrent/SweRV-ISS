// Copyright 2020 Western Digital Corporation or its affiliates.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//     http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <span>
#include "HartConfig.hpp"
#include "Args.hpp"
#include "Session.hpp"
#include "third_party/nlohmann/json.hpp"


using namespace WdRiscv;


#include <termios.h>


int
main(int argc, char* argv[])
{
  // Used to restore terminal state via RAII
  class TerminalStateRAII
  {
  public:
    TerminalStateRAII()
    { tcgetattr(STDIN_FILENO, &term); }

    ~TerminalStateRAII()
    { tcsetattr(STDIN_FILENO, 0, &term); }  // Restore terminal state.

  private:
    struct termios term;
  };

  bool ok = true;
  try
    {
      Args args;
      if (not args.parseCmdLineArgs(std::span(argv, argc)))
        return 1;
      if (args.help or args.version)
        return 0;

      // Load configuration file.
      HartConfig config;
      if (not args.configFile.empty())
        if (not config.loadConfigFile(args.configFile))
          return 1;

      TerminalStateRAII term;  // Save/restore terminal state.

      unsigned regWidth = Session<uint32_t>::determineRegisterWidth(args, config);

      if (regWidth == 32)
	{
	  Session<uint32_t> session{};
	  ok = session.defineSystem(args, config) != nullptr;
	  ok = ok and session.configureSystem(args, config);
	  ok = ok and session.run(args);
	}
      else if (regWidth == 64)
	{
	  Session<uint64_t> session{};
	  ok = session.defineSystem(args, config) != nullptr;
	  ok = ok and session.configureSystem(args, config);
	  ok = ok and session.run(args);
	}
      else
        {
          std::cerr << "Invalid register width: " << regWidth;
          std::cerr << " -- expecting 32 or 64\n";
          ok = false;
        }
    }
  catch (std::exception& e)
    {
      std::cerr << e.what() << '\n';
      ok = false;
    }

  return ok? 0 : 1;
}
