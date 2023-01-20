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

namespace WdRiscv
{

  /// Privilige mode.
  enum class PrivilegeMode : uint32_t
    {
      User = 0,
      Supervisor = 1,
      Reserved = 2,
      Machine = 3
    };


  /// RISCV interrupt cause.
  enum class InterruptCause : uint32_t
    {
      RESERVED0    = 0,  //
      S_SOFTWARE   = 1,  // Supervisor mode software interrupt
      VS_SOFTWARE  = 2,  // Virtual supervisor software interrupt
      M_SOFTWARE   = 3,  // Machine mode software interrupt
      RESERVED1    = 4,  // 
      S_TIMER      = 5,  // Supervisor timer
      M_TIMER      = 7,  // Machine timer
      RESERVED2    = 8,  // 
      S_EXTERNAL   = 9,  // Supervisor external
      VS_EXTERNAL  = 10, // Virtual supervisor external
      M_EXTERNAL   = 11, // Machine external
      M_INT_TIMER1 = 28, // Internal timer 1 (WD extension) bit position.
      M_INT_TIMER0 = 29, // Internal timer 0 (WD extension) bit position.
      M_LOCAL      = 30, // Correctable error local interrupt (WD extension)
      MAX_CAUSE    = M_LOCAL
    };


  /// RISCV exception cause.
  enum class ExceptionCause : uint32_t
    {
      INST_ADDR_MISAL        = 0,  // Instruction address misaligned
      INST_ACC_FAULT         = 1,  // Instruction access fault
      ILLEGAL_INST           = 2,  // Illegal instruction
      BREAKP                 = 3,  // Breakpoint
      LOAD_ADDR_MISAL        = 4,  // Load address misaligned
      LOAD_ACC_FAULT         = 5,  // Load access fault
      STORE_ADDR_MISAL       = 6,  // Store address misaligned
      STORE_ACC_FAULT        = 7,  // Store access fault.
      U_ENV_CALL             = 8,  // Environment call from user mode
      S_ENV_CALL             = 9,  // Environment call from supervisor mode
      VS_ENV_CALL            = 10, // Environment call from virtual supervisor mode
      M_ENV_CALL             = 11, // Environment call from machine mode
      INST_PAGE_FAULT        = 12, // Instruction page fault
      LOAD_PAGE_FAULT        = 13, // Load page fault
      STORE_PAGE_FAULT       = 15, // Store page fault
      RESERVED0              = 16,
      RESERVED1              = 17,
      RESERVED2              = 18,
      RESERVED3              = 19,
      INST_GUEST_PAGE_FAULT  = 20, // Instruction guest-page fault.
      LOAD_GUEST_PAGE_FAULT  = 21, // Load guest-page fault.
      VIRT_INST              = 22, // Virtual instruction
      STORE_GUEST_PAGE_FAULT = 23, // Store guest-page fault.
      NONE                   = 24,
      MAX_CAUSE              = NONE
    };


  /// RISCV trap vector mode.
  enum class TrapVectorMode : uint32_t
    {
      Direct             = 0,   // All traps set pc to BASE
      Vectored           = 1    // For exceptions, pc = BASE. For interrupts, pc = BASE + 4xcause
    };


  /// Non-maskable interrupt cause.
  enum class NmiCause : uint32_t
    {
      UNKNOWN               = 0,
      STORE_EXCEPTION       = 0xf0000000,
      LOAD_EXCEPTION        = 0xf0000001,
      DOUBLE_BIT_ECC        = 0xf0001000,
      DCCM_ACCESS_ERROR     = 0xf0001001,
      NON_DCCM_ACCESS_ERROR = 0xf0001002
    };


  /// Reason for entering debug mode (value stored in cause field
  /// of dcsr)
  enum class DebugModeCause
    {
      EBREAK = 1, TRIGGER = 2, DEBUGGER = 3, STEP = 4
    };

}
