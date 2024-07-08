// Copyright 2022 Tenstorrent Corporation or its affiliates.
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

#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include "FpRegs.hpp"
#include "VecRegs.hpp"
#include "DecodedInst.hpp"
#include "Decoder.hpp"
#include "Disassembler.hpp"
#include "float-util.hpp"
#include "instforms.hpp"


using namespace WdRiscv;


static constexpr
std::string_view
roundingModeString(RoundingMode mode)
{
  switch (mode)
    {
    case RoundingMode::NearestEven: return "rne";
    case RoundingMode::Zero:        return "rtz";
    case RoundingMode::Down:        return "rdn";
    case RoundingMode::Up:          return "rup";
    case RoundingMode::NearestMax:  return "rmm";
    case RoundingMode::Invalid1:    return "inv1";
    case RoundingMode::Invalid2:    return "inv2";
    case RoundingMode::Dynamic:     return "dyn";
    case RoundingMode::FcsrMask:    return "inv";
    }
  return "inv";
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form:  inst reg1, imm(reg2)
static
void
printLdSt(const Disassembler& disas, std::ostream& stream, const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << di.name() << ' ';

  const char* sign = imm < 0? "-" : "";
  if (imm < 0)
    imm = -imm;

  // Keep least sig 12 bits.
  imm = imm & 0xfff;

  stream << (di.isFp() ? disas.fpRegName(rd) : disas.intRegName(rd))
	 << ", " << sign << "0x" << std::hex << imm
	 << "(" << disas.intRegName(rs1) << ")" << std::dec;
}


/// Helper to disassemble method. Print on the given stream the
/// disassembly of the given instruction.
static
void
printInst(const Disassembler& disas, std::ostream& out, const DecodedInst& di)
{
  if (di.isLoad() or di.isStore())
    {
      printLdSt(disas, out, di);
      return;
    }

  std::string name = di.name();
  unsigned width = std::max(size_t(9), name.size() + 1);

  out << std::left << std::setw(static_cast<int>(width)) << name;
  unsigned opCount = di.operandCount();

  const char* sep = "";

  for (unsigned i = 0; i < opCount; ++i)
    {
      out << sep; sep = ", ";

      auto type = di.ithOperandType(i);
      switch (type)
	{
	case OperandType::IntReg:
	  out << disas.intRegName(di.ithOperand(i));
	  break;
	case OperandType::FpReg:
	  out << disas.fpRegName(di.ithOperand(i));
	  break;
	case OperandType::VecReg:
	  out << "v" << di.ithOperand(i);
	  break;
	case OperandType::Imm:
	  out << di.ithOperandAsInt(i);
	  break;
	default:
	  out << "??";
	  break;
	}
    }

  if (di.instEntry()->hasRoundingMode())
    out << sep << roundingModeString(RoundingMode(di.roundingMode()));
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: csrinst rd, csrn, rs1
static
void
printCsr(const Disassembler& disas, std::ostream& stream,
	 const DecodedInst& di)
{
  unsigned rd = di.op0(), csrn = di.op2();

  stream << std::left << std::setw(9) << di.name();
  stream << disas.intRegName(rd) << ", ";
  stream << disas.csRegName(csrn);

  if (di.ithOperandType(1) == OperandType::Imm)
    stream << ", 0x" << std::hex << di.op1() << std::dec;
  else
    stream << ", " << disas.intRegName(di.op1());
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg, imm where inst is a
/// compressed instruction.
static
void
printRegImm(const Disassembler& disas, std::ostream& stream,
	    const char* inst, unsigned rs1, int32_t imm)
{
  // Print instruction in a 8 character field.
  stream << std::left << std::setw(8) << inst << ' ';

  stream << disas.intRegName(rs1) << ", ";

  if (imm < 0)
    stream << "-0x" << std::hex << (-imm) << std::dec;
  else
    stream << "0x" << std::hex << imm << std::dec;
}


/// Helper to disassemble method. Print on the given stream given 3
/// operand branch instruction which is of the form: inst reg, reg,
/// imm where imm is a 12 bit constant.
static
void
printBranch3(const Disassembler& disas, std::ostream& stream,
	     const DecodedInst& di)
{
  unsigned rs1 = di.op0(), rs2 = di.op1();

  stream << std::left << std::setw(8) << di.name() << ' ';
  stream << disas.intRegName(rs1) << ", " << disas.intRegName(rs2) << ", . ";

  char sign = '+';
  int32_t imm = di.op2As<int32_t>();
  if (imm < 0)
    {
      sign = '-';
      imm = -imm;
    }

  stream << sign << " 0x" << std::hex << imm << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// 2 operand  branch instruction which is of the form: inst reg, imm.
static
void
printBranch2(const Disassembler& disas, std::ostream& stream, const DecodedInst& di)
{
  unsigned rs1 = di.op0();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << di.name() << ' ';
  stream << disas.intRegName(rs1) << ", . ";

  char sign = '+';
  if (imm < 0)
    {
      sign = '-';
      imm = -imm;
    }
  stream << sign << " 0x" << std::hex << imm << std::dec;
}


static
void
printFence(const Disassembler& , std::ostream& stream,
	   const DecodedInst& di)
{
  stream << std::left << std::setw(8) << di.name() << ' ';

  std::string pred, succ;

  if (di.isFencePredRead())   pred += "r";
  if (di.isFencePredWrite())  pred += "w";
  if (di.isFencePredInput())  pred += "i";
  if (di.isFencePredOutput()) pred += "o";

  if (di.isFenceSuccRead())   succ += "r";
  if (di.isFenceSuccWrite())  succ += "w";
  if (di.isFenceSuccInput())  succ += "i";
  if (di.isFenceSuccOutput()) succ += "o";

  if (pred.empty() and succ.empty())
    return;
  if (pred.empty())
    pred = "0";
  if (succ.empty())
    succ = "0";
  stream << pred << ", " << succ;
}


/// Helper to disassemble method.
static
void
printAmo(const Disassembler& disas, std::ostream& stream, const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << di.name();

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << disas.intRegName(rd) << ", " << disas.intRegName(rs2) << ", ("
	 << disas.intRegName(rs1) << ")";
}


/// Helper to disassemble method.
static
void
printLr(const Disassembler& disas, std::ostream& stream, const char* inst,
	const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << inst;

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << disas.intRegName(rd) << ", (" << disas.intRegName(rs1) << ")";
}


/// Helper to disassemble method.
static
void
printSc(const Disassembler& disas, std::ostream& stream, const char* inst,
	const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << inst;

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << disas.intRegName(rd) << ", " << disas.intRegName(rs2)
	 << ", (" << disas.intRegName(rs1) << ")";
}


static
void
printVecInst(const Disassembler& disas, std::ostream& out, const DecodedInst& di)
{
  uint32_t opcode7 = di.inst() & 0x7f;  // Least sig 7 bits
  InstId id = di.instId();

  if (opcode7 == 0x7 or opcode7 == 0x27)
    {  // Vector load store
      out << di.name() << " v" << di.op0();
      out << ", ("  << disas.intRegName(di.op1()) << ")";
      if (di.operandCount() == 3)
	{
	  if (di.ithOperandType(2) == OperandType::IntReg)
	    out << ", " << disas.intRegName(di.ithOperand(2));
	  else
	    out << ", v" << di.op2();
	}
      if (di.isMasked())
	out << ", v0.t";
      return;
    }

  if (id == InstId::vsetvli or id == InstId::vsetivli)
    {
      out << di.name() << ' ' << disas.intRegName(di.op0()) << ", ";
      if (id == InstId::vsetivli)
	out << di.op1();
      else
	out << disas.intRegName(di.op1());
      out << ", ";
      std::string mm = ((di.op2() >> 7) & 1) ? "ma" : "mu";
      std::string tt = ((di.op2() >> 6) & 1) ? "ta" : "tu";
      auto gm = VecRegs::to_string(GroupMultiplier(di.op2() & 7));
      auto ew = VecRegs::to_string(ElementWidth((di.op2() >> 3) & 7));
      out << ew << ',' << gm << ',' << tt << ',' << mm;
      return;
    }

  if (id == InstId::vsetvl)
    {
      out << "vsetvl " << disas.intRegName(di.op0()) << ", "
	  << disas.intRegName(di.op1()) << ", " << disas.intRegName(di.op2());
      return;
    }

  out << di.name();

  std::string_view sep = " ";

  for (unsigned i = 0; i < di.operandCount(); ++i)
    {
      out << sep; sep = ", ";

      auto type = di.ithOperandType(i);
      switch (type)
	{
	case OperandType::IntReg:
	  out << disas.intRegName(di.ithOperand(i));
	  break;
	case OperandType::FpReg:
	  out << disas.fpRegName(di.ithOperand(i));
	  break;
	case OperandType::VecReg:
	  out << "v" << di.ithOperand(i);
	  break;
	case OperandType::Imm:
	  out << di.ithOperandAsInt(i);
	  break;
	default:
	  out << "??";
	  break;
	}
    }

  if (di.isMasked())
    {
      if ((id >= InstId::vadc_vvm and id <= InstId::vmsbc_vxm) or
	  (id >= InstId::vmerge_vvm and id <= InstId::vmerge_vim) or
	  (id == InstId::vfmerge_vfm))
	out << sep << "v0";
      else
	out << sep << "v0.t";
    }
}


static
void
printCbo(const Disassembler& disas, std::ostream& out, const DecodedInst& di)
{
  std::string name = di.name();
  unsigned width = std::max(size_t(9), name.size() + 1);
  out << std::left << std::setw(static_cast<int>(width)) << name;
  out << "0(" << disas.intRegName(di.op0()) << ")";
}


static
void
printPrefetch(const Disassembler& disas, std::ostream& out, const DecodedInst& di)
{
  std::string name = di.name();
  unsigned width = std::max(size_t(9), name.size() + 1);
  out << std::left << std::setw(static_cast<int>(width)) << name;
  out << di.op1() << '(' << disas.intRegName(di.op0()) << ")";
}



static
void
printLfi(const Disassembler& disas, std::ostream& out, const DecodedInst& di)
{
  std::string name = di.name();
  out << std::left << std::setw(9) << name;
  out << disas.fpRegName(di.op0()) << ", ";
  const char* infOr64k = (di.instId() == InstId::fli_h) ? "inf" : "65536.0";
    
  switch(di.op1())
    {
    case 0:  out << "-1.0";           break;
    case 1:  out << "min";            break;
    case 2:  out << "1.52587891e-05"; break;
    case 3:  out << "3.05175781e-05"; break;
    case 4:  out << "0.00390625";     break;
    case 5:  out << "0.0078125";      break;
    case 6:  out << "0.0625";         break;
    case 7:  out << "0.125";          break;
    case 8:  out << "0.25";           break;
    case 9:  out << "0.3125";         break;
    case 10: out << "0.375";          break;
    case 11: out << "0.4375";         break;
    case 12: out << "0.5";            break;
    case 13: out << "0.625";          break;
    case 14: out << "0.75";           break;
    case 15: out << "0.875";          break;
    case 16: out << "1.0";            break;
    case 17: out << "1.25";           break;
    case 18: out << "1.5";            break;
    case 19: out << "1.75";           break;
    case 20: out << "2.0";            break;
    case 21: out << "2.5";            break;
    case 22: out << "3.0";            break;
    case 23: out << "4.0";            break;
    case 24: out << "8.0";            break;
    case 25: out << "16.0";           break;
    case 26: out << "128.0";          break;
    case 27: out << "256.0";          break;
    case 28: out << "32768.0";        break;
    case 29: out << "65536.0";        break;
    case 30: out << infOr64k;         break;
    case 31: out << "nan";            break;
    default : out << "?";             break;
    }
}


void
Disassembler::disassembleInst(uint32_t inst, const Decoder& decoder,
			      std::string& str)
{
  str.clear();
  DecodedInst di;
  decoder.decode(0, 0, inst, di);
  disassembleInst(di, str);
}


void
Disassembler::disassembleInst(const DecodedInst& di, std::string& str)
{
  str.clear();
  disassemble(di, str);
}


void
Disassembler::disassembleUncached(const DecodedInst& di, std::ostream& out) const
{
  InstId id = di.instId();
  switch(id)
    {
    case InstId::illegal:
      out << "illegal";
      break;

    case InstId::lui:
      printRegImm(*this, out, "lui", di.op0(), di.op1As<int32_t>() >> 12);
      break;

    case InstId::auipc:
      out << "auipc    " << intRegName(di.op0())
	  << ", 0x" << std::hex << ((di.op1() >> 12) & 0xfffff) << std::dec;
      break;

    case InstId::jal:
      {
	if (di.op0() == 0)
	  out << "j        ";
	else
	  out << "jal      " << intRegName(di.op0()) << ", ";
	char sign = '+';
	int32_t imm = di.op1As<int32_t>();
	if (imm < 0) { sign = '-'; imm = -imm; }
	out << ". " << sign << " 0x" << std::hex << (imm & 0xfffff) << std::dec;
      }
      break;

    case InstId::jalr:
      printLdSt(*this, out, di);
      break;

    case InstId::beq:
    case InstId::bne:
    case InstId::blt:
    case InstId::bge:
    case InstId::bltu:
    case InstId::bgeu:
      printBranch3(*this, out, di);
      break;

    case InstId::fence_tso:
    case InstId::fence:
      printFence(*this, out, di);
      break;

    case InstId::csrrw:
    case InstId::csrrs:
    case InstId::csrrc:
    case InstId::csrrwi:
    case InstId::csrrsi:
    case InstId::csrrci:
      printCsr(*this, out, di);
      break;

    case InstId::pack:
      if (not isRv64() and di.op2() == 0)
	out << "zext.h   " << intRegName(di.ithOperand(0)) << ", " << intRegName(di.ithOperand(1));
      else
	printInst(*this, out, di);
      break;

    case InstId::packw:
      if (di.op2() == 0)
	out << "zext.h   " << intRegName(di.ithOperand(0)) << ", " << intRegName(di.ithOperand(1));
      else
	printInst(*this, out, di);
      break;

    case InstId::lr_w:
      printLr(*this, out, "lr.w", di);
      break;

    case InstId::sc_w:
      printSc(*this, out, "sc.w", di);
      break;

    case InstId::lr_d:
      printLr(*this, out, "lr.d", di);
      break;

    case InstId::sc_d:
      printSc(*this, out, "sc.d", di);
      break;

    case InstId::c_addi4spn:
      printRegImm(*this, out, "c.addi4spn ", di.op0(), di.op2As<int32_t>() >> 2);
      break;

    case InstId::c_lq:
    case InstId::c_sq:
      out << "illegal";
      break;

    case InstId::c_addi:
      if (di.op0() == 0)
	out << "c.nop";
      else
	printRegImm(*this, out, "c.addi", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_jal:
      {
	out << "c.jal    . ";
	int32_t imm = di.op1As<int32_t>();
	char sign = '+';
	if (imm < 0) { sign = '-'; imm = -imm; }
	out << sign << " 0x" << std::hex << imm << std::dec;
      }
      break;

    case InstId::c_li:
      printRegImm(*this, out, "c.li", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_addi16sp:
      {
	int32_t imm = di.op2As<int32_t>();
	out << "c.addi16sp ";
	if (imm < 0) { out << "-"; imm = -imm; }
	out << "0x" << std::hex << (imm >> 4) << std::dec;
      }
      break;

    case InstId::c_lui:
      printRegImm(*this, out, "c.lui", di.op0(), static_cast<int32_t>(di.op1() >> 12));
      break;

    case InstId::c_srli:
      printRegImm(*this, out, "c.srli", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_srli64:
      printRegImm(*this, out, "c.srli64", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_srai:
      printRegImm(*this, out, "c.srai", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_srai64:
      printRegImm(*this, out, "c.srai64", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_andi:
      printRegImm(*this, out, "c.andi", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_sub:
      out << "c.sub    " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_xor:
      out << "c.xor    " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_or:
      out << "c.or     " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_and:
      out << "c.and    " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_subw:
      out << "c.subw   " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_addw:
      out << "c.addw   " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_j:
      {
	out << "c.j      . ";
	int32_t imm = di.op1As<int32_t>();
	char sign = '+';
	if (imm < 0) { sign = '-'; imm = -imm; }
	out << sign << " 0x" << std::hex << imm << std::dec;
      }
      break;

    case InstId::c_beqz:
    case InstId::c_bnez:
      printBranch2(*this, out, di);
      break;

    case InstId::c_slli:
      out << "c.slli   " << intRegName(di.op0()) << ", " << di.op2();
      break;

    case InstId::c_slli64:
      out << "c.slli64 " << intRegName(di.op0()) << ", " << di.op2();
      break;

    case InstId::c_fldsp:
      out << "c.fldsp   " << fpRegName(di.op0()) << ", 0x" << std::hex
	  << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_lwsp:
      out << "c.lwsp   " << intRegName(di.op0()) << ", 0x" << std::hex
	  << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_flwsp:
      out << "c.flwsp   " << fpRegName(di.op0()) << ", 0x" << std::hex
	  << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_ldsp:
      out << "c.ldsp   " << intRegName(di.op0()) << ", 0x" << std::hex
	  << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_jr:
      out << "c.jr     " << intRegName(di.op1());
      break;

    case InstId::c_mv:
      out << "c.mv     " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_ebreak:
      out << "c.ebreak";
      break;

    case InstId::c_jalr:
      out << "c.jalr   " << intRegName(di.op1());
      break;

    case InstId::c_add:
      out << "c.add    " << intRegName(di.op0()) << ", " << intRegName(di.op2());
      break;

    case InstId::c_fsdsp:
      out << "c.fsdsp   " << fpRegName(di.op0())
	  << ", 0x" << std::hex << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_swsp:
      out << "c.swsp   " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_fswsp:
      out << "c.swsp   " << fpRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_addiw:
      printRegImm(*this, out, "c.addiw", di.op0(), di.op2As<int32_t>());
      break;

    case InstId::c_sdsp:
      out << "c.sdsp   " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec;
      break;

    case InstId::c_lbu:
      out << "c.lbu    " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec
	  << "(" << intRegName(di.op1()) << ")";
      break;

    case InstId::c_lhu:
      out << "c.lhu    " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec
	  << "(" << intRegName(di.op1()) << ")";
      break;

    case InstId::c_lh:
      out << "c.lh     " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec
	  << "(" << intRegName(di.op1()) << ")";
      break;

    case InstId::c_sb:
      out << "c.sb     " << intRegName(di.op0()) << ", 0x"
	  << std::hex << di.op2As<int32_t>() << std::dec
	  << "(" << intRegName(di.op1()) << ")";
      break;

    case InstId::cbo_clean:
    case InstId::cbo_flush:
    case InstId::cbo_inval:
    case InstId::cbo_zero:
      printCbo(*this, out, di);
      break;

    case InstId::prefetch_i:
    case InstId::prefetch_r:
    case InstId::prefetch_w:
      printPrefetch(*this, out, di);
      break;

    case InstId::fli_h:
    case InstId::fli_s:
    case InstId::fli_d:
      printLfi(*this, out, di);
      break;

    default:
      if (di.instEntry()->isAtomic())
	printAmo(*this, out, di);
      else if (di.instEntry()->isVector())
	printVecInst(*this, out, di);
      else
	printInst(*this, out, di);
    }
}


void
Disassembler::disassemble(const DecodedInst& di, std::string& str)
{
  uint32_t inst = di.inst();
  auto iter = disasMap_.find(inst);
  if (iter != disasMap_.end())
    {
      str = iter->second;
    }
  else
    {
      std::ostringstream oss;
      disassembleUncached(di, oss);
      str = oss.str();
      disasMap_[di.inst()] = str;
    }
}
