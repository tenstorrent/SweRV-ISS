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
#include <sstream>
#include <fstream>
#include <map>
#include <algorithm>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "WhisperMessage.h"
#include "Hart.hpp"
#include "Server.hpp"
#include "Interactive.hpp"


using namespace WdRiscv;


/// Unpack socket message (received in server mode) into the given
/// WhisperMessage object.
static
void
deserializeMessage(const char buffer[], size_t bufferLen,
		   WhisperMessage& msg)

{
  assert (bufferLen >= sizeof(msg));

  const char* p = buffer;
  uint32_t x = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.hart = x;
  p += sizeof(x);

  x = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.type = x;
  p += sizeof(x);

  x = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.resource = x;
  p += sizeof(x);

  x = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.size = x;
  p += sizeof(x);

  x = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.flags = x;
  p += sizeof(x);

  uint32_t part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.instrTag = uint64_t(part) << 32;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.instrTag |= part;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.time = uint64_t(part) << 32;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.time |= part;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.address = uint64_t(part) << 32;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.address |= part;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.value = uint64_t(part) << 32;
  p += sizeof(part);

  part = ntohl(* reinterpret_cast<const uint32_t*> (p));
  msg.value |= part;
  p += sizeof(part);

  memcpy(msg.buffer, p, sizeof(msg.buffer));
  p += sizeof(msg.buffer);

  memcpy(msg.tag, p, sizeof(msg.tag));
  p += sizeof(msg.tag);

  assert(size_t(p - buffer) <= bufferLen);
}


/// Serialize the given WhisperMessage into the given buffer in
/// preparation for socket send. Return the number of bytes written
/// into buffer.
static
size_t
serializeMessage(const WhisperMessage& msg, char buffer[],
		 size_t bufferLen)
{
  assert (bufferLen >= sizeof(msg));

  char* p = buffer;
  uint32_t x = htonl(msg.hart);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  x = htonl(msg.type);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  x = htonl(msg.resource);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  x = htonl(msg.size);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  x = htonl(msg.flags);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  uint32_t part = static_cast<uint32_t>(msg.instrTag >> 32);
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = (msg.instrTag) & 0xffffffff;
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = static_cast<uint32_t>(msg.time >> 32);
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = (msg.time) & 0xffffffff;
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = static_cast<uint32_t>(msg.address >> 32);
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = (msg.address) & 0xffffffff;
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = static_cast<uint32_t>(msg.value >> 32);
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  part = msg.value & 0xffffffff;
  x = htonl(part);
  memcpy(p, &x, sizeof(x));
  p += sizeof(x);

  memcpy(p, msg.buffer, sizeof(msg.buffer));
  p += sizeof(msg.buffer);

  memcpy(p, msg.tag, sizeof(msg.tag));
  p += sizeof(msg.tag);

  size_t len = p - buffer;
  assert(len <= bufferLen);
  assert(len <= sizeof(msg));
  for (size_t i = len; i < sizeof(msg); ++i)
    buffer[i] = 0;

  return sizeof(msg);
}


static bool
receiveMessage(int soc, WhisperMessage& msg)
{
  char buffer[sizeof(msg)];
  char* p = buffer;

  size_t remain = sizeof(msg);

  while (remain > 0)
    {
      ssize_t l = recv(soc, p, remain, 0);
      if (l < 0)
	{
	  if (errno == EINTR)
	    continue;
	  std::cerr << "Failed to receive socket message\n";
	  return false;
	}
      if (l == 0)
	{
	  msg.type = Quit;
	  return true;
	}
      remain -= l;
      p += l;
    }

  deserializeMessage(buffer, sizeof(buffer), msg);

  return true;
}


static bool
sendMessage(int soc, WhisperMessage& msg)
{
  char buffer[sizeof(msg)];

  serializeMessage(msg, buffer, sizeof(buffer));

  // Send command.
  ssize_t remain = sizeof(msg);
  char* p = buffer;
  while (remain > 0)
    {
      ssize_t l = send(soc, p, remain , MSG_NOSIGNAL);
      if (l < 0)
	{
	  if (errno == EINTR)
	    continue;
	  std::cerr << "Failed to send socket command\n";
	  return false;
	}
      remain -= l;
      p += l;
    }

  return true;
}


static bool
receiveMessage(char* shm, WhisperMessage& msg)
{
  // reserve first byte for locking
  std::atomic_char* guard = (std::atomic_char*) shm;
  while (std::atomic_load(guard) != 's');

  deserializeMessage(shm + 1, sizeof(msg), msg);
  return true;
}


static bool
sendMessage(char* shm, WhisperMessage& msg)
{
  // reserve first byte for locking
  std::atomic_char* guard = (std::atomic_char*) shm;
  while (std::atomic_load(guard) != 's'); // redundant

  serializeMessage(msg, shm + 1, sizeof(msg));
  std::atomic_store(guard, 'c');
  return true;
}


template <typename URV>
Server<URV>::Server(System<URV>& system)
  : system_(system)
{
}
  

template <typename URV>
bool
Server<URV>::pokeCommand(const WhisperMessage& req, WhisperMessage& reply)
{
  reply = req;

  if (not checkHartId(req, reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  switch (req.resource)
    {
    case 'r':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	URV val = static_cast<URV>(req.value);
	if (reg == req.address)
	  if (hart.pokeIntReg(reg, val))
	    return true;
      }
      break;

    case 'f':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	uint64_t val = req.value;
	if (reg == req.address)
	  if (hart.pokeFpReg(reg, val))
	    return true;
      }
      break;

    case 'c':
      {
	URV val = static_cast<URV>(req.value);
	if (hart.externalPokeCsr(CsrNumber(req.address), val))
	  return true;
      }
      break;

    case 'm':
      {
        bool usePma = false; // Ignore phsical memory attributes.

        if (sizeof(URV) == 4)
          {
            // Poke a word in 32-bit harts.
            if (hart.pokeMemory(req.address, uint32_t(req.value), usePma))
              return true;
          }
        else if (hart.pokeMemory(req.address, req.value, usePma))
          return true;
      }
      break;

    case 'p':
      {
	URV val = static_cast<URV>(req.value);
	hart.pokePc(val);
	return true;
      }
      break;

    case 's':
      {
        bool ok = true;
        URV val = static_cast<URV>(req.value);
        if (req.address == WhisperSpecialResource::DeferredInterrupts)
          hart.setDeferredInterrupts(val);
        else
          ok = false;
        if (ok)
          return true;
        else
          break;
      }
    }

  reply.type = Invalid;
  return false;
}


template <typename URV>
bool
Server<URV>::peekCommand(const WhisperMessage& req, WhisperMessage& reply)
{
  reply = req;

  if (not checkHartId(req, reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  URV value;

  switch (req.resource)
    {
    case 'r':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	if (reg == req.address)
	  if (hart.peekIntReg(reg, value))
	    {
	      reply.value = value;
	      return true;
	    }
      }
      break;
    case 'f':
      {
	unsigned reg = static_cast<unsigned>(req.address);
	uint64_t fpVal = 0;
	if (reg == req.address)
	  if (hart.peekFpReg(reg, fpVal))
	    {
	      reply.value = fpVal;
	      return true;
	    }
      }
      break;
    case 'c':
      if (hart.peekCsr(CsrNumber(req.address), value))
	{
	  reply.value = value;
	  return true;
	}
      break;
    case 'm':
      if (hart.peekMemory(req.address, value, false /*usePma*/))
	{
	  reply.value = value;
	  return true;
	}
      break;
    case 'p':
      reply.value = hart.peekPc();
      return true;
    case 's':
      {
	bool ok = true;
	if (req.address == WhisperSpecialResource::PrivMode)
	  reply.value = unsigned(hart.privilegeMode());
	else if (req.address == WhisperSpecialResource::PrevPrivMode)
	  reply.value = unsigned(hart.lastPrivMode());
	else if (req.address == WhisperSpecialResource::FpFlags)
	  reply.value = hart.lastFpFlags();
	else if (req.address == WhisperSpecialResource::Trap)
	  reply.value = hart.lastInstructionTrapped()? 1 : 0;
        else if (req.address == WhisperSpecialResource::DeferredInterrupts)
          reply.value = hart.deferredInterrupts();
	else
	  ok = false;
        if (ok)
          return true;
        else
          break;
      }
    case 'i':
      {
        uint32_t inst;
        if (hart.readInst(req.address, inst))
          {
            reply.value = inst;
            return true;
          }
      }
    }

  reply.type = Invalid;
  return true;
}


template <typename URV>
void
Server<URV>::disassembleAnnotateInst(Hart<URV>& hart,
                                     const DecodedInst& di, bool interrupted,
				     bool hasPreTrigger, bool hasPostTrigger,
				     std::string& text)
{
  hart.disassembleInst(di.inst(), text);
  if (di.isBranch())
    {
      if (hart.lastPc() + di.instSize() != hart.peekPc())
       text += " (T)";
      else
       text += " (NT)";
    }

  if (not interrupted)
    {
      uint64_t va = 0, pa = 0;
      if (hart.lastLdStAddress(va, pa))
	{
	  std::ostringstream oss;
	  oss << " [0x" << std::hex << va << "]" << std::dec;
	  text += oss.str();
	}
    }

  if (interrupted)
    text += " (interrupted)";
  else if (hasPreTrigger)
    text += " (pre-trigger)";
  else if (hasPostTrigger)
    text += " (post-trigger)";
}


/// Collect the double-words overlapping the areas of memory modified
/// by the most recent emulated system call. It is assumed that
/// memory protection boundaries are double-word aligned.
template <typename URV>
static void
collectSyscallMemChanges(Hart<URV>& hart,
                         const std::vector<std::pair<uint64_t, uint64_t>>& scVec,
                         std::vector<WhisperMessage>& changes,
                         uint64_t slamAddr)
{
  for (auto al : scVec)
    {
      uint64_t addr = al.first;
      uint64_t len = al.second;

      uint64_t offset = addr & 7;  // Offset from double word address.
      if (offset)
        {
          addr -= offset;   // Make double word aligned.
          len += offset;    // Keep last byte the same.
        }

      for (uint64_t ix = 0; ix < len; ix += 8, addr += 8)
        {
          uint64_t val = 0;
          if (not hart.peekMemory(addr, val, true))
            {
              std::cerr << "Peek-memory fail at 0x" << std::hex << addr << std::dec
                        << " in collectSyscallMemChanges\n";
              break;
            }

          if (not slamAddr)
            continue;

          bool ok = hart.pokeMemory(slamAddr, addr, true);
          if (ok)
            {
              changes.push_back(WhisperMessage{0, Change, 'm', slamAddr, addr, 8});

              slamAddr += 8;
              ok = hart.pokeMemory(slamAddr, val, true);
              if (ok)
                {
                  changes.push_back(WhisperMessage{0, Change, 'm', slamAddr, val, 8});
                  slamAddr += 8;
                }
            }

          if (not ok)
            {
              std::cerr << "Poke-memory fail at 0x" << std::hex << slamAddr << std::dec
                        << " in collectSyscallMemChanges\n";
              break;
            }
        }
    }

  // Put a zero to mark end of syscall changes in slam area.
  if (slamAddr)
    {
      if (hart.pokeMemory(slamAddr, uint64_t(0), true))
        changes.push_back(WhisperMessage{0, Change, 'm', slamAddr, 0});
      if (hart.pokeMemory(slamAddr + 8, uint64_t(0), true))
        changes.push_back(WhisperMessage{0, Change, 'm', slamAddr+8, 0});
    }
}


template <typename URV>
void
Server<URV>::processStepCahnges(Hart<URV>& hart,
				uint32_t inst,
				std::vector<WhisperMessage>& pendingChanges,
				bool interrupted, bool hasPre, bool hasPost,
				WhisperMessage& reply)
{
  // Get executed instruction.
  URV pc = hart.lastPc();

  // Add pc and instruction to reply.
  reply.type = ChangeCount;
  reply.address = pc;
  reply.resource = inst;

  // Add disassembly of instruction to reply.
  DecodedInst di;
  hart.decode(0 /*addr: fake*/, 0 /*physAddr: fake*/, inst, di);
  std::string text;
  disassembleAnnotateInst(hart, di, interrupted, hasPre, hasPost, text);

  strncpy(reply.buffer, text.c_str(), sizeof(reply.buffer) - 1);
  reply.buffer[sizeof(reply.buffer) -1] = 0;

  // Order of changes: rfcvm (int reg, fp reg, csr, vec reg, memory, csr)

  // Collect integer register change caused by execution of instruction.
  pendingChanges.clear();
  uint64_t lastVal = 0;
  int regIx = hart.lastIntReg(lastVal);
  if (regIx > 0)
    {
      URV value = 0;
      if (hart.peekIntReg(regIx, value))
	{
	  WhisperMessage msg;
	  msg.type = Change;
	  msg.resource = 'r';
	  msg.address = regIx;
	  msg.value = value;
	  msg.size = sizeof(msg.value);
	  msg.time = lastVal;  // Re-purpose otherwise unused time field.
	  pendingChanges.push_back(msg);
	}
    }

  // Collect floating point register change.
  int fpRegIx = hart.lastFpReg(lastVal);
  if (fpRegIx >= 0)
    {
      uint64_t val = 0;
      if (hart.peekFpReg(fpRegIx, val))
	{
	  WhisperMessage msg;
	  msg.type = Change;
	  msg.resource = 'f';
	  msg.address = fpRegIx;
	  msg.value = val;
	  msg.size = sizeof(msg.value);
	  msg.time = lastVal;  // Re-purpose otherwise unused time field.
	  pendingChanges.push_back(msg);
	}
    }

  // Collect vector register change.
  unsigned groupSize = 0;
  int vecReg = hart.lastVecReg(di, groupSize);
  if (vecReg >= 0)
    {
      for (unsigned ix = 0; ix < groupSize; ++ix, ++vecReg)
	{
	  std::vector<uint8_t> vecData;
	  if (not hart.peekVecReg(vecReg, vecData))
	    assert(0 && "Failed to peek vec register");

	  // Reverse bytes since peekVecReg returns most significant
	  // byte first.
	  std::reverse(vecData.begin(), vecData.end());

	  // Send a change message for each vector element starting
	  // with element zero and assuming a vector of double words
	  // (uint64_t). Last element will be padded with zeros if
	  // vector size in bytes is not a multiple of 8.
	  unsigned byteCount = vecData.size();
	  for (unsigned byteIx = 0; byteIx < byteCount; )
	    {
	      WhisperMessage msg;
	      msg.type = Change;
	      msg.resource = 'v';
	      msg.address = vecReg;

	      unsigned size = sizeof(msg.value);
	      unsigned remain = byteCount - byteIx;
	      size = std::min(size, remain);
	      msg.size = size;

	      for (unsigned i = 0; i < size; ++i)
		msg.value |= uint64_t(vecData.at(byteIx++)) << (8*i);
	      pendingChanges.push_back(msg);
	    }
	}
    }

  // Collect CSR and trigger changes.
  std::vector<CsrNumber> csrs;
  std::vector<unsigned> triggers;
  hart.lastCsr(csrs, triggers);

  // Map to keep CSRs in order and to drop duplicate entries.
  std::map<URV,URV> csrMap;

  // Components of the triggers that changed (if any).
  std::vector<bool> tdataChanged(3);

  // Collect changed CSRs and their values. Collect components of
  // changed trigger.
  for (CsrNumber csr : csrs)
    {
      URV value;
      if (hart.peekCsr(csr, value))
	{
	  if (csr >= CsrNumber::TDATA1 and csr <= CsrNumber::TDATA3)
	    {
	      size_t ix = size_t(csr) - size_t(CsrNumber::TDATA1);
	      tdataChanged.at(ix) = true;
	    }
	  else
	    csrMap[URV(csr)] = value;
	}
    }

  // Collect changes associated with trigger register.
  for (unsigned trigger : triggers)
    {
      uint64_t data1(0), data2(0), data3(0);
      if (not hart.peekTrigger(trigger, data1, data2, data3))
	continue;

      // Components of trigger that changed.
      bool t1 = false, t2 = false, t3 = false;
      hart.getTriggerChange(trigger, t1, t2, t3);

      if (t1)
	{
	  URV addr = (trigger << 16) | unsigned(CsrNumber::TDATA1);
	  csrMap[addr] = data1;
	}
      if (t2)
	{
	  URV addr = (trigger << 16) | unsigned(CsrNumber::TDATA2);
	  csrMap[addr] = data2;
	}
      if (t3)
	{
	  URV addr = (trigger << 16) | unsigned(CsrNumber::TDATA3);
	  csrMap[addr] = data3;
	}
    }

  for (const auto& [key, val] : csrMap)
    {
      WhisperMessage msg(0, Change, 'c', key, val);
      msg.size = sizeof(msg.value);
      pendingChanges.push_back(msg);
    }

  // Collect memory change.
  uint64_t memAddr = 0, memVal = 0;
  unsigned size = hart.lastStore(memAddr, memVal);
  if (size)
    {
      WhisperMessage msg(0, Change, 'm', memAddr, memVal, size);
      pendingChanges.push_back(msg);
    }
  else
    {
      std::vector<uint64_t> addr;
      std::vector<uint64_t> data;
      unsigned elemSize = 0;
      if (hart.getLastVectorMemory(addr, data, elemSize) and not data.empty())
	for (size_t i = 0; i < data.size(); ++i)
	  {
	    WhisperMessage msg(0, Change, 'm', addr.at(i), data.at(i), elemSize);
	    pendingChanges.push_back(msg);
	  }
    }

  // Collect emulated system call changes.
  uint64_t slamAddr = hart.syscallSlam();
  if (slamAddr)
    {
      std::vector<std::pair<uint64_t, uint64_t>> scVec;
      hart.lastSyscallChanges(scVec);
      if (scVec.size())
        collectSyscallMemChanges(hart, scVec, pendingChanges, slamAddr);
    }

  // Add count of changes to reply.
  reply.value = pendingChanges.size();

  // The changes will be retrieved one at a time from the back of the
  // pendigChanges vector: Put the vector in reverse order. Changes
  // are retrieved using a Change request (see interactUsingSocket).
  std::reverse(pendingChanges.begin(), pendingChanges.end());
}


template <typename URV>
bool
Server<URV>::checkHartId(const WhisperMessage& req, WhisperMessage& reply)
{
  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    {
      std::cerr << "Error: Hart ID (" << std::dec << hartId
                << ") out of bounds\n";
      reply.type = Invalid;
      return false;
    }
  return true;
}


template <typename URV>
bool
Server<URV>::checkHart(const WhisperMessage& req, const std::string& /*command*/,
                       WhisperMessage& reply)
{
  if (not checkHartId(req, reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;

  return true;
}


// Server mode step command.
template <typename URV>
bool
Server<URV>::stepCommand(const WhisperMessage& req, 
			 std::vector<WhisperMessage>& pendingChanges,
			 WhisperMessage& reply,
			 FILE* traceFile)
{
  reply = req;

  // Hart id must be valid. Hart must be started.
  if (not checkHart(req, "step", reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  // Get instruction before execution (in case code is self-modifying).
  uint32_t inst = 0;
  hart.readInst(hart.peekPc(), inst);

  unsigned pm = unsigned(hart.privilegeMode());

  // Execute instruction. Determine if an interrupt was taken or if a
  // trigger got tripped.
  uint64_t interruptCount = hart.getInterruptCount();

  bool wasInDebug = hart.inDebugMode();
  if (wasInDebug)
    hart.exitDebugMode();

  // Memory consistency model support. No-op if mcm is off.
  if (system_.isMcmEnabled())
    {
      system_.mcmSetCurrentInstruction(hart, req.instrTag);
      hart.setInstructionCount(req.instrTag - 1);
      DecodedInst di;
      hart.singleStep(di, traceFile);
      if (not di.isValid())
	assert(hart.lastInstructionTrapped());
      system_.mcmRetire(hart, req.time, req.instrTag, di);
    }
  else
    hart.singleStep(traceFile);

  bool interrupted = hart.getInterruptCount() != interruptCount;

  unsigned preCount = 0, postCount = 0;
  hart.countTrippedTriggers(preCount, postCount);

  bool hasPre = preCount > 0;
  bool hasPost = postCount > 0;

  processStepCahnges(hart, inst, pendingChanges, interrupted, hasPre,
		     hasPost, reply);

  // Send privilege mode, incremental floating point flags, and trap info
  // in flags: 2 bits for priv mode, 4 bits for fp flags, 1 bit for trap,
  // 1 bit if target program stopped.
  unsigned fpFlags = hart.lastFpFlags();
  unsigned trap = hart.lastInstructionTrapped()? 1 : 0;
  unsigned stop = hart.hasTargetProgramFinished()? 1 : 0;
  reply.flags = (pm & 3) | ((fpFlags & 0xf) << 2) | (trap << 6) | (stop << 7);;

  if (wasInDebug)
    hart.enterDebugMode(hart.peekPc());
  return true;
}


// Server mode step command.
template <typename URV>
bool
Server<URV>::translateCommand(const WhisperMessage& req, 
			      WhisperMessage& reply)
{
  reply = req;

  // Hart id must be valid. Hart must be started.
  if (not checkHart(req, "step", reply))
    return false;

  uint32_t hartId = req.hart;
  auto hartPtr = system_.findHartByHartId(hartId);
  if (not hartPtr)
    return false;
  auto& hart = *hartPtr;

  uint64_t va = req.address;
  bool r = req.flags & 1, w = (req.flags & 2) != 0, x = (req.flags & 4) != 0;
  PrivilegeMode pm = (req.flags & 8) ? PrivilegeMode::Supervisor : PrivilegeMode::User;

  uint64_t pa = 0;
  auto ec = hart.transAddrNoUpdate(va, pm, r, w, x, pa);
  if (ec != ExceptionCause::NONE)
    {
      reply.type = Invalid;
      return false;
    }

  reply.address = pa;
  return true;
}


/// Dump all registers contents in tracefile.
template <typename URV>
static void
serverPrintFinalRegisterState(std::shared_ptr<Hart<URV>> hartPtr)
{
  std::ofstream out("issfinal.log");
  if (not out)
    return;
  Interactive<URV>::peekAllIntRegs(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllFpRegs(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllTriggers(*hartPtr, out);
  out << "\n";
  Interactive<URV>::peekAllCsrs(*hartPtr, out);
}


static
const char*
specialResourceToStr(uint64_t v)
{
  WhisperSpecialResource sr = WhisperSpecialResource(v);
  switch (sr)
    {
    case WhisperSpecialResource::PrivMode:            return "pm";
    case WhisperSpecialResource::PrevPrivMode:        return "ppm";
    case WhisperSpecialResource::FpFlags:             return "iff";
    case WhisperSpecialResource::Trap:                return "trap";
    case WhisperSpecialResource::DeferredInterrupts:  return "defi";
    default:                                          return "?";
    }
  return "?";
}


template <typename URV>
void
doPageTableWalk(const Hart<URV>& hart, WhisperMessage& reply)
{
  bool isInstr = reply.flags & 1;
  bool isAddr = reply.flags & 2;
  unsigned index = reply.address;

  std::vector<uint64_t> items;
  if (isAddr)
    hart.getPageTableWalkAddresses(isInstr, index, items);
  else
    hart.getPageTableWalkEntries(isInstr, index, items);
  if (items.size() > 5)
    {
      std::cerr << "doPageTableWalk: Walk too long: " << items.size() << " entries\n";
      reply.type = Invalid;
      return;
    }

  reply.size = items.size();
  if (items.size())
    memcpy(reply.buffer, items.data(), items.size()*sizeof(uint64_t));
}


// Server mode loop: Receive command and send reply till a quit
// command is received. Return true on successful termination (quit
// received). Return false otherwise.
template <typename URV>
bool
Server<URV>::interact(int soc, FILE* traceFile, FILE* commandLog)
{
  while (true)
    {
      WhisperMessage msg, reply;
      if (not receiveMessage(soc, msg))
	return false;

      interact(msg, reply, traceFile, commandLog);

      if (not sendMessage(soc, reply))
	return false;
    }

  return false;
}


template <typename URV>
bool
Server<URV>::interact(char* shm, FILE* traceFile, FILE* commandLog)
{
  while (true)
    {
      WhisperMessage msg, reply;
      if (not receiveMessage(shm, msg))
	return false;

      interact(msg, reply, traceFile, commandLog);

      if (not sendMessage(shm, reply))
	return false;
    }

  return false;
}


template <typename URV>
bool
Server<URV>::interact(const WhisperMessage& msg, WhisperMessage& reply, FILE* traceFile, FILE* commandLog)
{
  // Initial resets do not reset memory mapped registers.
  bool resetMemoryMappedReg = false;
  reply = msg;

  if (checkHartId(msg, reply))
    {
      std::string timeStamp = std::to_string(msg.time);

      uint32_t hartId = msg.hart;
      auto hartPtr = system_.findHartByHartId(hartId);
      assert(hartPtr);
      auto& hart = *hartPtr;

      if (msg.type == Step or msg.type == Until)
        resetMemoryMappedReg = true;

      switch (msg.type)
        {
        case Quit:
          if (commandLog)
            fprintf(commandLog, "hart=%d quit\n", hartId);
          serverPrintFinalRegisterState(hartPtr);
          return true;

        case Poke:
          pokeCommand(msg, reply);
          if (commandLog)
            {
              if (msg.resource == 'p')
                fprintf(commandLog, "hart=%d poke pc 0x%jx # ts=%s tag=%s\n", hartId,
                        uintmax_t(msg.value), timeStamp.c_str(), msg.tag);
              else
                fprintf(commandLog, "hart=%d poke %c 0x%jx 0x%jx # ts=%s tag=%s\n", hartId,
                        msg.resource, uintmax_t(msg.address), uintmax_t(msg.value),
                        timeStamp.c_str(), msg.tag);
            }
          break;

        case Peek:
          peekCommand(msg, reply);
          if (commandLog)
            {
              if (msg.resource == 'p')
                fprintf(commandLog, "hart=%d peek pc # ts=%s tag=%s\n",
                        hartId, timeStamp.c_str(), msg.tag);
              else if (msg.resource == 's')
                fprintf(commandLog, "hart=%d peek s %s # ts=%s tag=%s\n",
                        hartId, specialResourceToStr(msg.address),
                        timeStamp.c_str(), msg.tag);
              else
                fprintf(commandLog, "hart=%d peek %c 0x%jx # ts=%s tag=%s\n",
                        hartId, msg.resource, uintmax_t(msg.address),
                        timeStamp.c_str(), msg.tag);
            }
          break;

        case Step:
          stepCommand(msg, pendingChanges_, reply, traceFile);
          if (commandLog)
            {
              if (system_.isMcmEnabled())
                fprintf(commandLog, "hart=%d time=%s step 1 %jd\n",
                        hartId, timeStamp.c_str(), uintmax_t(msg.instrTag));
              else
                fprintf(commandLog, "hart=%d step #%jd # ts=%s\n",
                        hartId, uintmax_t(hart.getInstructionCount()),
                        timeStamp.c_str());
              fflush(commandLog);
            }
          break;

        case ChangeCount:
          reply.type = ChangeCount;
          reply.value = pendingChanges_.size();
          reply.address = hart.lastPc();
          {
            uint32_t inst = 0;
            hart.readInst(hart.lastPc(), inst);
            reply.resource = inst;
            std::string text;
            hart.disassembleInst(inst, text);
            uint32_t op0 = 0, op1 = 0, op2 = 0, op3 = 0;
            const InstEntry& entry = hart.decode(inst, op0, op1, op2, op3);
            if (entry.isBranch())
              {
                if (hart.lastPc() + instructionSize(inst) != hart.peekPc())
                  text += " (T)";
                else
                  text += " (NT)";
              }
            strncpy(reply.buffer, text.c_str(), sizeof(reply.buffer) - 1);
            reply.buffer[sizeof(reply.buffer) -1] = 0;
          }
          break;

        case Change:
          if (pendingChanges_.empty())
            reply.type = Invalid;
          else
            {
              reply = pendingChanges_.back();
              pendingChanges_.pop_back();
            }
          break;

        case Reset:
          {
            URV addr = static_cast<URV>(msg.address);
            if (addr != msg.address)
              std::cerr << "Error: Address too large (" << std::hex
                        << msg.address << ") in reset command.\n" << std::dec;
            pendingChanges_.clear();
            if (msg.value != 0)
              hart.defineResetPc(addr);
            hart.reset(resetMemoryMappedReg);
            if (commandLog)
              {
                if (msg.value != 0)
                  fprintf(commandLog, "hart=%d reset 0x%jx # ts=%s\n", hartId,
                          uintmax_t(addr), timeStamp.c_str());
                else
                  fprintf(commandLog, "hart=%d reset # ts=%s\n", hartId,
                          timeStamp.c_str());
              }
          }
          break;

        case EnterDebug:
          {
            if (checkHart(msg, "enter_debug", reply))
              hart.enterDebugMode(hart.peekPc());
            if (commandLog)
              fprintf(commandLog, "hart=%d enter_debug # ts=%s\n", hartId,
                      timeStamp.c_str());
          }
          break;

        case ExitDebug:
          if (checkHart(msg, "exit_debug", reply))
            hart.exitDebugMode();
          if (commandLog)
            fprintf(commandLog, "hart=%d exit_debug # ts=%s\n", hartId,
                    timeStamp.c_str());
          break;

        case CancelDiv:
          if (checkHart(msg, "cancel_div", reply))
            if (not hart.cancelLastDiv())
              reply.type = Invalid;
          if (commandLog)
            fprintf(commandLog, "hart=%d cancel_div # ts=%s\n", hartId,
                    timeStamp.c_str());
          break;

        case CancelLr:
          if (checkHart(msg, "cancel_lr", reply))
            hart.cancelLr();
          if (commandLog)
            fprintf(commandLog, "hart=%d cancel_lr # ts=%s\n", hartId,
                    timeStamp.c_str());
          break;

        case DumpMemory:
          if (not system_.writeAccessedMemory(msg.buffer))
            reply.type = Invalid;
          if (commandLog)
            fprintf(commandLog, "hart=%d dump_memory %s # ts=%s\n",
                    hartId, msg.buffer, timeStamp.c_str());
          break;

        case McmRead:
          if (not system_.mcmRead(hart, msg.time, msg.instrTag, msg.address,
                                  msg.size, msg.value, msg.flags))
            reply.type = Invalid;
          if (commandLog)
            fprintf(commandLog, "hart=%d time=%ld mread %ld 0x%lx %d 0x%lx %s\n",
                    hartId, msg.time, msg.instrTag, msg.address, msg.size,
                    msg.value, msg.flags? "i" : "e");
          break;

        case McmInsert:
          if (not system_.mcmMbInsert(hart, msg.time, msg.instrTag,
                                      msg.address, msg.size, msg.value))
            reply.type = Invalid;
          if (commandLog)
            fprintf(commandLog, "hart=%d time=%ld mbinsert %ld 0x%lx %d 0x%lx\n",
                    hartId, msg.time, msg.instrTag, msg.address, msg.size,
                    msg.value);
          break;

        case McmWrite:
          if (msg.size > sizeof(msg.buffer))
            {
              std::cerr << "Error: Server command: McmWrite data size too large: " << msg.size << '\n';
              reply.type = Invalid;
            }
          else
            {
              std::vector<bool> mask;
              if (msg.flags)
                mask.resize(msg.size);

              std::vector<uint8_t> data(msg.size);
              for (size_t i = 0; i < msg.size; ++i)
                {
                  data.at(i) = msg.buffer[i];
                  if (msg.flags)
                    mask.at(i) = msg.tag[i/8] & (1 << (i%8));
                }

              if (not system_.mcmMbWrite(hart, msg.time, msg.address, data, mask))
                reply.type = Invalid;

              if (commandLog)
                {
                  fprintf(commandLog, "hart=%d time=%ld mbwrite 0x%lx 0x",
                          hartId, msg.time, msg.address);
		  std::reverse(data.begin(), data.end()); // Print lowest address data on the right.
                  for (uint8_t item :  data)
                    fprintf(commandLog, "%02x", item);
                  if (msg.flags)
                    {
                      fprintf(commandLog, " 0x");
		      data.clear();
                      for (size_t i = 0; i < msg.size / 8; ++i)
			data.push_back(msg.tag[i]);
		      std::reverse(data.begin(), data.end());
		      for (uint8_t item : data)
                        fprintf(commandLog, "%02x", item);
                    }
                  fprintf(commandLog, "\n");
                }
            }
          break;

        case PageTableWalk:
          doPageTableWalk(hart, reply);
          break;

	case Translate:
	  translateCommand(msg, reply);
          if (commandLog)
	    {
	      auto flags = msg.flags;
	      const char* rwx = "r";
	      if (flags & 1) rwx = "r";
	      else if (flags & 2) rwx = "w";
	      else if (flags & 4) rwx = "x";
	      const char* su = (flags & 8) ? "s" : "u";
	      fprintf(commandLog, "hart=%d translate 0x%jx %s %s\n", hartId,
		      uintmax_t(msg.address), rwx, su);
	    }
	  break;

	case CheckInterrupt:
	  {
	    URV mipVal = msg.address;
	    InterruptCause cause = InterruptCause{0};
	    reply.flags = hart.isInterruptPossible(mipVal, cause);
	    if (commandLog)
	      fprintf(commandLog, "hart=%d check_interrupt 0x%jx\n", hartId,
		      uintmax_t(msg.value));
	  }
	  break;

        default:
          std::cerr << "Unknown command\n";
          reply.type = Invalid;
        }
    }

  return false;
}


template class WdRiscv::Server<uint32_t>;
template class WdRiscv::Server<uint64_t>;
