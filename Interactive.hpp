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

#pragma once

#include <cstdint>
#include <iosfwd>
#include <unordered_map>
#include <vector>
#include "util.hpp"


namespace WdRiscv
{

  template <typename URV>
  class Hart;

  template <typename URV>
  class System;

  /// Manage an interactive session. To use: Construct an instance
  /// with one or more harts then invoke the interact method which
  /// will read commands from the standard input and execute them
  /// until the quit command is seen. URV (unsigned register value) is
  /// either uint32_t or uint64_t depending on the integer register
  /// width of the harts.
  template <typename URV>
  class Interactive
  {
  public:

    /// Constructor.
    Interactive(System<URV>& system);

    /// Read commands from the standard input and execute them.
    /// Instance traces go the the given traceFile (no instance
    /// tracing if traceFile is NULL). Executed commands are logged to
    /// the give commandLog file (no comand logging if commandLog is
    /// NULL). Return true if all commands are executed successfully.
    /// Return false otherwise.
    bool interact(FILE* traceFile, FILE* commandLog);

    /// Helper to interact: "until" command. Run until address. If the hart is
    /// in debug mode (halted), this is also a resume.
    bool untilCommand(Hart<URV>&, const std::string& line,
		     const std::vector<std::string>& tokens,
		     FILE* traceFile);

    /// Helper to interact: "run" command. If the hart is in debug
    /// mode (halted), this is also a resume.
    bool runCommand(Hart<URV>&, const std::string& line,
		    const std::vector<std::string>& tokens,
		    FILE* traceFile);

    /// Helper to interact: "step" command. Single step. If the hart is in debug
    /// mode it will be resumed for one instruction and will re-enter debug
    /// mode immediately after regardless of dcsr.step.
    bool stepCommand(Hart<URV>&, const std::string& line,
		     const std::vector<std::string>& tokens, FILE* traceFile);

    /// Helper to interact: "peek" command. Examine a register/memory
    /// location.
    bool peekCommand(Hart<URV>&, const std::string& line,
		     const std::vector<std::string>& tokens,
                     std::ostream& out);

    /// Helper to interact: "poke" command. Set a register/memory
    /// location.
    bool pokeCommand(Hart<URV>&, const std::string& line,
		     const std::vector<std::string>& tokens);

    /// Helper to interact: "disass" command. Disassemble.
    bool disassCommand(Hart<URV>&, const std::string& line,
		       const std::vector<std::string>& tokens);

    /// Helper to interact: "elf" command. Load ELF file.
    bool elfCommand(Hart<URV>&, const std::string& line,
		    const std::vector<std::string>& tokens);

    /// Helper to interact: "hex" command. Load HEX file.
    bool hexCommand(Hart<URV>&, const std::string& line,
		    const std::vector<std::string>& tokens);

    /// Helper to interact: "lz4" command. Load LZ4 compresed binary file.
    bool lz4Command(Hart<URV>& , const std::string& line,
		    const std::vector<std::string>& tokens);

    /// Helper to interact: "reset" command. Reset processor.
    bool resetCommand(Hart<URV>&, const std::string& line,
		     const std::vector<std::string>& tokens);

    /// Helper to interact: "replay_file" command. Define replay file.
    bool replayFileCommand(const std::string& line,
			   const std::vector<std::string>& tokens,
			   std::ifstream& stream);

    /// Helper to interact: "dump_memory" command.
    bool dumpMemoryCommand(const std::string& line,
                           const std::vector<std::string>& tokens);

    /// Helper to interact: "help" command.
    void helpCommand(const std::vector<std::string>& tokens);

    /// Helper to interact: "replay" command. Replay one or more
    /// commands from the replay file.
    bool replayCommand(const std::string& line,
		       const std::vector<std::string>& tokens,
		       FILE* traceFile, FILE* commandLog,
		       std::ifstream& replayStream, bool& done);

    bool mreadCommand(Hart<URV>& hart, const std::string& line,
		     const std::vector<std::string>& tokens);

    bool mbwriteCommand(Hart<URV>& hart, const std::string& line,
			const std::vector<std::string>& tokens);

    bool mbinsertCommand(Hart<URV>& hart, const std::string& line,
			 const std::vector<std::string>& tokens);

    bool mbbypassCommand(Hart<URV>& hart, const std::string& line,
			 const std::vector<std::string>& tokens);

    bool mifetchCommand(Hart<URV>& hart, const std::string& line,
			const std::vector<std::string>& tokens);

    bool mievictCommand(Hart<URV>& hart, const std::string& line,
			const std::vector<std::string>& tokens);

    bool translateCommand(Hart<URV>& hart, const std::string& line,
			  const std::vector<std::string>& tokens);

    bool checkInterruptCommand(Hart<URV>& hart, const std::string& line,
			       const std::vector<std::string>& tokens);

    bool seiPinCommand(Hart<URV>& hart, const std::string& line,
		       const std::vector<std::string>& tokens);

    bool pmpCommand(Hart<URV>& hart, const std::string& line,
		    const std::vector<std::string>& tokens);

    bool pmaCommand(Hart<URV>& hart, const std::string& line,
		    const std::vector<std::string>& tokens);

    bool perfModelFetchCommand(const std::string& line,
		               const std::vector<std::string>& tokens);

    bool perfModelDecodeCommand(const std::string& line,
		                const std::vector<std::string>& tokens);

    bool perfModelExecuteCommand(const std::string& line,
		                 const std::vector<std::string>& tokens);

    bool perfModelRetireCommand(const std::string& line,
		                const std::vector<std::string>& tokens,
                                FILE* traceFile);

    bool perfModelDrainStoreCommand(const std::string& line,
		                    const std::vector<std::string>& tokens);

    static void peekAllFpRegs(Hart<URV>& hart, std::ostream& out);
    static void peekAllIntRegs(Hart<URV>& hart, std::ostream& out);
    static void peekAllVecRegs(Hart<URV>& hart, std::ostream& out);
    static void peekAllCsrs(Hart<URV>& hart, std::ostream& out);
    static void peekAllTriggers(Hart<URV>& hart, std::ostream& out);

  protected:

    /// Helper to interact. Execute a user command.
    bool executeLine(const std::string& inLine, FILE* traceFile,
		     FILE* commandLog,
		     std::ifstream& replayStream, bool& done);

    using StringMap = std::unordered_map<std::string, std::string, util::string_hash, std::equal_to<>>;

    /// Process time=<number>, and/or hart=<number>
    /// tokens present in an interactive command. Update time_,
    /// and/or hart_ accordingly. Return true on success and
    /// false on error.
    bool processKeywords(const StringMap& strMap);

  private:

    System<URV>& system_;

    // Initial resets do not reset memory mapped registers.
    bool resetMemoryMappedRegs_ = false;

    uint64_t time_ = 0;
    uint64_t hartId_ = 0;
  };

}
