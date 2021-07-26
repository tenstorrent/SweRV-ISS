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

#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include "Hart.hpp"
#include "instforms.hpp"


using namespace WdRiscv;


static
std::string
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
    default:                        return "inv";
    }
  return "inv";
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rd, rs1, rs2
template <typename URV>
static
void
printRdRs1Rs2(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	      const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;
  if (strlen(inst) >= 9)
    stream << ' ';

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1) << ", "
	 << hart.intRegName(rs2);
}


/// Helper to disassemble method. Print on the given stream given
/// 2-operand floating point instruction.
template <typename URV>
static
void
printFp2(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	 const DecodedInst& di)
{
  stream << std::left << std::setw(9) << inst << hart.fpRegName(di.op0())
	 << ", " << hart.fpRegName(di.op1());
}


/// Helper to disassemble method. Print on the given stream given
/// 3-operand floating point instruction.
template <typename URV>
static
void
printFp3(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	 const DecodedInst& di)
{
  stream << std::left << std::setw(9) << inst << hart.fpRegName(di.op0())
	 << ", " << hart.fpRegName(di.op1())
	 << ", " << hart.fpRegName(di.op2());
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rd, rs1
template <typename URV>
static
void
printRdRs1(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	   const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1);
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rd, rs2, rs1, rs3
template <typename URV>
static
void
printRdRs2Rs1Rs3(const Hart<URV>& hart, std::ostream& stream, const char* inst,
                 const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2(), rs3 = di.op3();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs2)
         << ", " << hart.intRegName(rs1) << ", " << hart.intRegName(rs3);
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rd, rs1, rs3, rs2
template <typename URV>
static
void
printRdRs1Rs3Rs2(const Hart<URV>& hart, std::ostream& stream, const char* inst,
                 const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2(), rs3 = di.op3();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1)
         << ", " << hart.intRegName(rs3) << ", " << hart.intRegName(rs2);
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rd, rs1, rs3, immed
template <typename URV>
static
void
printRdRs1Rs3Imm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
                   const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs3 = di.op2();
  unsigned imm = di.op3();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1)
         << ", " << hart.intRegName(rs3) << ", 0x" << std::hex << imm
         << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst rs1, rs2
template <typename URV>
static
void
printRs1Rs2(const Hart<URV>& hart, std::ostream& stream, const char* inst,
            const DecodedInst& di)
{
  unsigned rs1 = di.op0(), rs2 = di.op1();

  // Print instruction in a 9 character field.
  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rs1) << ", " << hart.intRegName(rs2) << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: csrinst rd, csrn, rs1
template <typename URV>
static
void
printCsr(Hart<URV>& hart, std::ostream& stream, const char* inst,
	 const DecodedInst& di)
{
  unsigned rd = di.op0(), csrn = di.op2();

  stream << std::left << std::setw(9) << inst;

  stream << hart.intRegName(rd) << ", ";

  auto csr = hart.findCsr(CsrNumber(csrn));
  if (csr)
    stream << csr->getName();
  else
    stream << "illegal";

  if (di.ithOperandType(1) == OperandType::Imm)
    stream << ", 0x" << std::hex << di.op1() << std::dec;
  else
    stream << ", " << hart.intRegName(di.op1());
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form:  inst reg1, imm(reg2)
template <typename URV>
static
void
printLdSt(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	  const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << inst << ' ';

  const char* sign = imm < 0? "-" : "";
  if (imm < 0)
    imm = -imm;

  // Keep least sig 12 bits.
  imm = imm & 0xfff;

  stream << hart.intRegName(rd) << ", " << sign << "0x"
	 << std::hex << imm << "(" << hart.intRegName(rs1) << ")" << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg1, imm(reg2) where inst
/// is a floating point ld/st instruction.
template <typename URV>
static
void
printFpLdSt(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	    const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << inst << ' ';

  const char* sign = imm < 0? "-" : "";
  if (imm < 0)
    imm = -imm;

  // Keep least sig 12 bits.
  imm = imm & 0xfff;

  stream << hart.fpRegName(rd) << ", " << sign << "0x" << std::hex << imm
	 << std::dec << "(" << hart.intRegName(rs1) << ")";
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg, reg, imm where inst is
/// a shift instruction.
template <typename URV>
static
void
printShiftImm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	      const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << inst << ' ';
  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1)
	 << ", 0x" << std::hex << imm << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg, reg, imm where imm is
/// a 12 bit constant.
template <typename URV>
static
void
printRegRegImm12(const Hart<URV>& hart, std::ostream& stream, const char* inst,
		 const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1) << ", ";

  if (imm < 0)
    stream << "-0x" << std::hex << ((-imm) & 0xfff) << std::dec;
  else
    stream << "0x" << std::hex << (imm & 0xfff) << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg, reg, uimm where uimm is
/// a 12 bit constant.
template <typename URV>
static
void
printRegRegUimm12(const Hart<URV>& hart, std::ostream& stream, const char* inst,
		  const DecodedInst& di)
{
  uint32_t rd = di.op0(), rs1 = di.op1(), imm = di.op2();

  stream << std::left << std::setw(8) << inst << ' ';
  stream << hart.intRegName(rd) << ", " << hart.intRegName(rs1) << ", ";
  stream << "0x" << std::hex << (imm & 0xfff) << std::dec;
}


/// Helper to disassemble method. Print on the given stream given
/// instruction which is of the form: inst reg, imm where inst is a
/// compressed instruction.
template <typename URV>
static
void
printRegImm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	    unsigned rs1, int32_t imm)
{
  // Print instruction in a 8 character field.
  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.intRegName(rs1) << ", ";

  if (imm < 0)
    stream << "-0x" << std::hex << (-imm) << std::dec;
  else
    stream << "0x" << std::hex << imm << std::dec;
}


/// Helper to disassemble method. Print on the given stream given 3
/// operand branch instruction which is of the form: inst reg, reg,
/// imm where imm is a 12 bit constant.
template <typename URV>
static
void
printBranch3(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	     const DecodedInst& di)
{
  unsigned rs1 = di.op0(), rs2 = di.op1();

  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.intRegName(rs1) << ", " << hart.intRegName(rs2) << ", . ";

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
template <typename URV>
static
void
printBranch2(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	     const DecodedInst& di)
{
  unsigned rs1 = di.op0();
  int32_t imm = di.op2As<int32_t>();

  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.intRegName(rs1) << ", . ";

  char sign = '+';
  if (imm < 0)
    {
      sign = '-';
      imm = -imm;
    }
  stream << sign << " 0x" << std::hex << imm << std::dec;
}


/// Helper to disassemble method.
template <typename URV>
static
void
printAmo(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	 const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << inst;

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << hart.intRegName(rd) << ", " << hart.intRegName(rs2) << ", ("
	 << hart.intRegName(rs1) << ")";
}


/// Helper to disassemble method.
template <typename URV>
static
void
printLr(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << inst;

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << hart.intRegName(rd) << ", (" << hart.intRegName(rs1) << ")";
}


/// Helper to disassemble method.
template <typename URV>
static
void
printSc(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	const DecodedInst& di)
{
  unsigned rd = di.op0(), rs1 = di.op1(), rs2 = di.op2();
  bool aq = di.isAtomicAcquire(), rl = di.isAtomicRelease();

  stream << inst;

  if (aq)
    stream << ".aq";

  if (rl)
    stream << ".rl";

  stream << ' ' << hart.intRegName(rd) << ", " << hart.intRegName(rs2)
	 << ", (" << hart.intRegName(rs1) << ")";
}


/// Helper to disassemble methods. Print a floating point instruction
/// with 4 operands and the rounding mode.
template <typename URV>
static
void
printFp4Rm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	   const DecodedInst& di)
{
  // Print instruction in a 8 character field.
  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.fpRegName(di.op0()) << ", " << hart.fpRegName(di.op1())
	 << ", " << hart.fpRegName(di.op2()) << ", " << hart.fpRegName(di.op3())
	 << ", " << roundingModeString(di.roundingMode());
}


/// Helper to disassemble methods. Print a floating point instruction
/// with 3 operands and the rounding mode.
template <typename URV>
static
void
printFp3Rm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	   const DecodedInst& di)
{
  // Print instruction in a 8 character field.
  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.fpRegName(di.op0()) << ", " << hart.fpRegName(di.op1())
	 << ", " << hart.fpRegName(di.op2())
	 << ", " << roundingModeString(di.roundingMode());
}


/// Helper to disassemble methods. Print a floating point instruction
/// with 2 operands and the rounding mode.
template <typename URV>
static
void
printFp2Rm(const Hart<URV>& hart, std::ostream& stream, const char* inst,
	   const DecodedInst& di)
{
  // Print instruction in a 8 character field.
  stream << std::left << std::setw(8) << inst << ' ';

  stream << hart.fpRegName(di.op0()) << ", " << hart.fpRegName(di.op1())
	 <<  ", " << roundingModeString(di.roundingMode());
}


template <typename URV>
static
void
printVec_vv(Hart<URV>& , std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", v" << di.op1() << ", v" << di.op2();
  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_vx(Hart<URV>& hart, std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", v" << di.op1() << ", "
      << hart.intRegName(di.op2());
  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_xv(Hart<URV>& hart, std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", " << hart.intRegName(di.op1())
      << ", v" << di.op2();
  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_vi(Hart<URV>& , std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", v" << di.op1()
      << ", 0x" << std::hex << di.op2As<int32_t>() << std::dec;
  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_vim(Hart<URV>& hart, std::ostream& out, const char* inst,
	     const DecodedInst& di)
{
  out << inst << " v v" << di.op0() << ", v" << di.op1()
      << ", 0x" << std::hex << di.op2As<int32_t>() << std::dec;
  if (di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_vf(Hart<URV>& hart, std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", v" << di.op1() << ", "
      << hart.fpRegName(di.op2());
  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
static
void
printVec_fv(Hart<URV>& hart, std::ostream& out, const char* inst,
	    const DecodedInst& di, bool mask = true)
{
  out << inst << " v" << di.op0() << ", " << hart.fpRegName(di.op1())
      << ", v" << di.op2();

  if (mask and di.isMasked())
    out << ", v0.t";
}


template <typename URV>
void
Hart<URV>::disassembleInst(uint32_t inst, std::ostream& stream)
{
  DecodedInst di;
  decode(pc_, inst, di);
  disassembleInst(di, stream);
}


template <typename URV>
void
Hart<URV>::disassembleInst(uint32_t inst, std::string& str)
{
  str.clear();

  std::ostringstream oss;
  disassembleInst(inst, oss);
  str = oss.str();
}


template <typename URV>
void
Hart<URV>::disassembleInst(const DecodedInst& di, std::ostream& out)
{
  InstId id = di.instEntry()->instId();
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
      printLdSt(*this, out, "jalr", di);
      break;

    case InstId::beq:
      printBranch3(*this, out, "beq",  di);
      break;

    case InstId::bne:
      printBranch3(*this, out, "bne",  di);
      break;

    case InstId::blt:
      printBranch3(*this, out, "blt",  di);
      break;

    case InstId::bge:
      printBranch3(*this, out, "bge",  di);
      break;

    case InstId::bltu:
      printBranch3(*this, out, "bltu",  di);
      break;

    case InstId::bgeu:
      printBranch3(*this, out, "bgeu",  di);
      break;

    case InstId::lb:
      printLdSt(*this, out, "lb",  di);
      break;

    case InstId::lh:
      printLdSt(*this, out, "lh",  di);
      break;

    case InstId::lw:
      printLdSt(*this, out, "lw",  di);
      break;

    case InstId::lbu:
      printLdSt(*this, out, "lbu",  di);
      break;

    case InstId::lhu:
      printLdSt(*this, out, "lhu",  di);
      break;

    case InstId::sb:
      printLdSt(*this, out, "sb", di);
      break;

    case InstId::sh:
      printLdSt(*this, out, "sh", di);
      break;

    case InstId::sw:
      printLdSt(*this, out, "sw", di);
      break;

    case InstId::addi:
      printRegRegImm12(*this, out, "addi", di);
      break;

    case InstId::slti:
      printRegRegImm12(*this, out, "slti", di);
      break;

    case InstId::sltiu:
      printRegRegUimm12(*this, out, "sltiu", di);
      break;

    case InstId::xori:
      printRegRegImm12(*this, out, "xori", di);
      break;

    case InstId::ori:
      printRegRegImm12(*this, out, "ori", di);
      break;

    case InstId::andi:
      printRegRegImm12(*this, out, "andi", di);
      break;

    case InstId::slli:
      printShiftImm(*this, out, "slli", di);
      break;

    case InstId::srli:
      printShiftImm(*this, out, "srli", di);
      break;

    case InstId::srai:
      printShiftImm(*this, out, "srai", di);
      break;

    case InstId::add:
      printRdRs1Rs2(*this, out, "add", di);
      break;

    case InstId::sub:
      printRdRs1Rs2(*this, out, "sub", di);
      break;

    case InstId::sll:
      printRdRs1Rs2(*this, out, "sll", di);
      break;

    case InstId::slt:
      printRdRs1Rs2(*this, out, "slt", di);
      break;

    case InstId::sltu:
      printRdRs1Rs2(*this, out, "sltu", di);
      break;

    case InstId::xor_:
      printRdRs1Rs2(*this, out, "xor", di);
      break;

    case InstId::srl:
      printRdRs1Rs2(*this, out, "srl", di);
      break;

    case InstId::sra:
      printRdRs1Rs2(*this, out, "sra", di);
      break;

    case InstId::or_:
      printRdRs1Rs2(*this, out, "or", di);
      break;

    case InstId::and_:
      printRdRs1Rs2(*this, out, "and", di);
      break;

    case InstId::fence:
      out << "fence";
      break;

    case InstId::fencei:
      out << "fencei";
      break;

    case InstId::ecall:
      out << "ecall";
      break;

    case InstId::ebreak:
      out << "ebreak";
      break;

    case InstId::csrrw:
      printCsr(*this, out, "csrrw", di);
      break;

    case InstId::csrrs:
      printCsr(*this, out, "csrrs", di);
      break;

    case InstId::csrrc:
      printCsr(*this, out, "csrrc", di);
      break;

    case InstId::csrrwi:
      printCsr(*this, out, "csrrwi", di);
      break;

    case InstId::csrrsi:
      printCsr(*this, out, "csrrsi", di);
      break;

    case InstId::csrrci:
      printCsr(*this, out, "csrrci", di);
      break;

    case InstId::lwu:
      printLdSt(*this, out, "lwu", di);
      break;

    case InstId::ld:
      printLdSt(*this, out, "ld", di);
      break;

    case InstId::sd:
      printLdSt(*this, out, "sd", di);
      break;

    case InstId::addiw:
      printRegRegImm12(*this, out, "addiw", di);
      break;

    case InstId::slliw:
      printShiftImm(*this, out, "slliw", di);
      break;

    case InstId::srliw:
      printShiftImm(*this, out, "srliw", di);
      break;

    case InstId::sraiw:
      printShiftImm(*this, out, "sraiw", di);
      break;

    case InstId::addw:
      printRdRs1Rs2(*this, out, "addw", di);
      break;

    case InstId::subw:
      printRdRs1Rs2(*this, out, "subw", di);
      break;

    case InstId::sllw:
      printRdRs1Rs2(*this, out, "sllw", di);
      break;

    case InstId::srlw:
      printRdRs1Rs2(*this, out, "srlw", di);
      break;

    case InstId::sraw:
      printRdRs1Rs2(*this, out, "sraw", di);
      break;

    case InstId::mul:
      printRdRs1Rs2(*this, out, "mul", di);
      break;

    case InstId::mulh:
      printRdRs1Rs2(*this, out, "mulh", di);
      break;

    case InstId::mulhsu:
      printRdRs1Rs2(*this, out, "mulhsu", di);
      break;

    case InstId::mulhu:
      printRdRs1Rs2(*this, out, "mulhu", di);
      break;

    case InstId::div:
      printRdRs1Rs2(*this, out, "div", di);
      break;

    case InstId::divu:
      printRdRs1Rs2(*this, out, "divu", di);
      break;

    case InstId::rem:
      printRdRs1Rs2(*this, out, "rem", di);
      break;

    case InstId::remu:
      printRdRs1Rs2(*this, out, "remu", di);
      break;

    case InstId::mulw:
      printRdRs1Rs2(*this, out, "mulw", di);
      break;

    case InstId::divw:
      printRdRs1Rs2(*this, out, "divw", di);
      break;

    case InstId::divuw:
      printRdRs1Rs2(*this, out, "divuw", di);
      break;

    case InstId::remw:
      printRdRs1Rs2(*this, out, "remw", di);
      break;

    case InstId::remuw:
      printRdRs1Rs2(*this, out, "remuw", di);
      break;

    case InstId::lr_w:
      printLr(*this, out, "lr.w", di);
      break;

    case InstId::sc_w:
      printSc(*this, out, "sc.w", di);
      break;

    case InstId::amoswap_w:
      printAmo(*this, out, "amoswap.w", di);
      break;

    case InstId::amoadd_w:
      printAmo(*this, out, "amoadd.w", di);
      break;

    case InstId::amoxor_w:
      printAmo(*this, out, "amoxor.w", di);
      break;

    case InstId::amoand_w:
      printAmo(*this, out, "amoand.w", di);
      break;

    case InstId::amoor_w:
      printAmo(*this, out, "amoor.w", di);
      break;

    case InstId::amomin_w:
      printAmo(*this, out, "amomin.w", di);
      break;

    case InstId::amomax_w:
      printAmo(*this, out, "amomax.w", di);
      break;

    case InstId::amominu_w:
      printAmo(*this, out, "amominu.w", di);
      break;

    case InstId::amomaxu_w:
      printAmo(*this, out, "amomaxu.w", di);
      break;

    case InstId::lr_d:
      printLr(*this, out, "lr.d", di);
      break;

    case InstId::sc_d:
      printSc(*this, out, "sc.d", di);
      break;

    case InstId::amoswap_d:
      printAmo(*this, out, "amoswap.d", di);
      break;

    case InstId::amoadd_d:
      printAmo(*this, out, "amoadd.d", di);
      break;

    case InstId::amoxor_d:
      printAmo(*this, out, "amoxor.d", di);
      break;

    case InstId::amoand_d:
      printAmo(*this, out, "amoand.d", di);
      break;

    case InstId::amoor_d:
      printAmo(*this, out, "amoor.d", di);
      break;

    case InstId::amomin_d:
      printAmo(*this, out, "amomin.d", di);
      break;

    case InstId::amomax_d:
      printAmo(*this, out, "amomax.d", di);
      break;

    case InstId::amominu_d:
      printAmo(*this, out, "amominu.d", di);
      break;

    case InstId::amomaxu_d:
      printAmo(*this, out, "amomaxu.d", di);
      break;

    case InstId::flw:
      printFpLdSt(*this, out, "flw", di);
      break;

    case InstId::fsw:
      printFpLdSt(*this, out, "fsw", di);
      break;

    case InstId::fmadd_s:
      printFp4Rm(*this, out, "fmadd.s", di);
      break;

    case InstId::fmsub_s:
      printFp4Rm(*this, out, "fmsub.s", di);
      break;

    case InstId::fnmsub_s:
      printFp4Rm(*this, out, "fnmsub.s", di);
      break;

    case InstId::fnmadd_s:
      printFp4Rm(*this, out, "fnmadd.s", di);
      break;

    case InstId::fadd_s:
      printFp3Rm(*this, out, "fadd.s", di);
      break;

    case InstId::fsub_s:
      printFp3Rm(*this, out, "fsub.s", di);
      break;

    case InstId::fmul_s:
      printFp3Rm(*this, out, "fmul.s", di);
      break;

    case InstId::fdiv_s:
      printFp3Rm(*this, out, "fdiv.s", di);
      break;

    case InstId::fsqrt_s:
      printFp2Rm(*this, out, "fsqrt.s", di);
      break;

    case InstId::fsgnj_s:
      printFp3(*this, out, "fsgnj.s", di);
      break;

    case InstId::fsgnjn_s:
      printFp3(*this, out, "fsgnjn.s", di);
      break;

    case InstId::fsgnjx_s:
      printFp3(*this, out, "fsgnjx.s", di);
      break;

    case InstId::fmin_s:
      printFp3(*this, out, "fmin.s", di);
      break;

    case InstId::fmax_s:
      printFp3(*this, out, "fmax.s", di);
      break;

    case InstId::fcvt_w_s:
      out << "fcvt.w.s "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_wu_s:
      out << "fcvt.wu.s " << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fmv_x_w:
      out << "fmv.x.w  " << intRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::feq_s:
      out << "feq.s    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::flt_s:
      out << "flt.s    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fle_s:
      out << "fle.s    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fclass_s:
      out << "fclass.s " << intRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::fcvt_s_w:
      out << "fcvt.s.w " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_s_wu:
      out << "fcvt.s.wu " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fmv_w_x:
      out << "fmv.w.x  " << fpRegName(di.op0()) << ", " << intRegName(di.op1());
      break;

    case InstId::fcvt_l_s:
      out << "fcvt.l.s "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_lu_s:
      out << "fcvt.lu.s "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_s_l:
      out << "fcvt.s.l " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_s_lu:
      out << "fcvt.s.lu " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fld:
      printFpLdSt(*this, out, "fld", di);
      break;

    case InstId::fsd:
      printFpLdSt(*this, out, "fsd", di);
      break;

    case InstId::fmadd_d:
      printFp4Rm(*this, out, "fmadd.d", di);
      break;

    case InstId::fmsub_d:
      printFp4Rm(*this, out, "fmsub.d", di);
      break;

    case InstId::fnmsub_d:
      printFp4Rm(*this, out, "fnmsub.d", di);
      break;

    case InstId::fnmadd_d:
      printFp4Rm(*this, out, "fnmadd.d", di);
      break;

    case InstId::fadd_d:
      printFp3Rm(*this, out, "fadd.d", di);
      break;

    case InstId::fsub_d:
      printFp3Rm(*this, out, "fsub.d", di);
      break;

    case InstId::fmul_d:
      printFp3Rm(*this, out, "fmul.d", di);
      break;

    case InstId::fdiv_d:
      printFp3Rm(*this, out, "fdiv.d", di);
      break;

    case InstId::fsqrt_d:
      printFp2Rm(*this, out, "fsqrt.d", di);
      break;

    case InstId::fsgnj_d:
      printFp3(*this, out, "fsgnj.d", di);
      break;

    case InstId::fsgnjn_d:
      printFp3(*this, out, "fsgnjn.d", di);
      break;

    case InstId::fsgnjx_d:
      printFp3(*this, out, "fsgnjx.d", di);
      break;

    case InstId::fmin_d:
      printFp3(*this, out, "fmin.d", di);
      break;

    case InstId::fmax_d:
      printFp3(*this, out, "fmax.d", di);
      break;

    case InstId::fcvt_s_d:
      out << "fcvt.s.d "  << fpRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_d_s:
      out << "fcvt.d.s "  << fpRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::feq_d:
      out << "feq.d    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::flt_d:
      out << "flt.d    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fle_d:
      out << "fle.d    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fclass_d:
      out << "fclass.d " << intRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::fcvt_w_d:
      out << "fcvt.w.d "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_wu_d:
      out << "fcvt.wu.d "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_d_w:
      out << "fcvt.d.w " << fpRegName(di.op0()) << ", " << intRegName(di.op1())
	  << ", " << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_d_wu:
      out << "fcvt.d.wu " << fpRegName(di.op0()) << ", " << intRegName(di.op1());
      break;

    case InstId::fcvt_l_d:
      out << "fcvt.l.d "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_lu_d:
      out << "fcvt.lu.d "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fmv_x_d:
      out << "fmv.x.d "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1());
      break;

    case InstId::fcvt_d_l:
      out << "fcvt.d.l " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1());
      break;

    case InstId::fcvt_d_lu:
      out << "fcvt.d.lu " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1());
      break;

    case InstId::fmv_d_x:
      out << "fmv.d.x " << fpRegName(di.op0()) << ", " << intRegName(di.op1());
      break;

    case InstId::flh:
      printFpLdSt(*this, out, "flh", di);
      break;

    case InstId::fsh:
      printFpLdSt(*this, out, "fsh", di);
      break;

    case InstId::fmadd_h:
      printFp4Rm(*this, out, "fmadd.h", di);
      break;

    case InstId::fmsub_h:
      printFp4Rm(*this, out, "fmsub.h", di);
      break;

    case InstId::fnmsub_h:
      printFp4Rm(*this, out, "fnmsub.h", di);
      break;

    case InstId::fnmadd_h:
      printFp4Rm(*this, out, "fnmadd.h", di);
      break;

    case InstId::fadd_h:
      printFp3Rm(*this, out, "fadd.h", di);
      break;

    case InstId::fsub_h:
      printFp3Rm(*this, out, "fsub.h", di);
      break;

    case InstId::fmul_h:
      printFp3Rm(*this, out, "fmul.h", di);
      break;

    case InstId::fdiv_h:
      printFp3Rm(*this, out, "fdiv.h", di);
      break;

    case InstId::fsqrt_h:
      printFp2Rm(*this, out, "fsqrt.h", di);
      break;

    case InstId::fsgnj_h:
      printFp3(*this, out, "fsgnj.h", di);
      break;

    case InstId::fsgnjn_h:
      printFp3(*this, out, "fsgnjn.h", di);
      break;

    case InstId::fsgnjx_h:
      printFp3(*this, out, "fsgnjx.h", di);
      break;

    case InstId::fmin_h:
      printFp3(*this, out, "fmin.h", di);
      break;

    case InstId::fmax_h:
      printFp3(*this, out, "fmax.h", di);
      break;

    case InstId::fcvt_s_h:
      printFp2(*this, out, "fcvt.s.h", di);
      break;

    case InstId::fcvt_d_h:
      printFp2(*this, out, "fcvt.d.h", di);
      break;

    case InstId::fcvt_h_s:
      printFp2(*this, out, "fcvt.h.s", di);
      break;

    case InstId::fcvt_h_d:
      printFp2(*this, out, "fcvt.h.d", di);
      break;

    case InstId::fcvt_w_h:
      out << "fcvt.w.h "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_wu_h:
      out << "fcvt.wu.h "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fmv_x_h:
      out << "fmv.x.h  " << intRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::feq_h:
      out << "feq.h    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::flt_h:
      out << "flt.h    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fle_h:
      out << "fle.h    " << intRegName(di.op0()) << ", " << fpRegName(di.op1())
	  << ", " << fpRegName(di.op2());
      break;

    case InstId::fclass_h:
      out << "fclass.h " << intRegName(di.op0()) << ", " << fpRegName(di.op1());
      break;

    case InstId::fcvt_h_w:
      out << "fcvt.h.w " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_h_wu:
      out << "fcvt.h.wu " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fmv_h_x:
      out << "fmv.h.x  " << fpRegName(di.op0()) << ", " << intRegName(di.op1());
      break;

    case InstId::fcvt_l_h:
      out << "fcvt.l.h "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_lu_h:
      out << "fcvt.lu.h "  << intRegName(di.op0()) << ", "
	  << fpRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_h_l:
      out << "fcvt.h.l " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::fcvt_h_lu:
      out << "fcvt.h.lu " << fpRegName(di.op0()) << ", "
	  << intRegName(di.op1()) << ", "
	  << roundingModeString(di.roundingMode());
      break;

    case InstId::mret:
      out << "mret";
      break;

    case InstId::uret:
      out << "uret";
      break;

    case InstId::sret:
      out << "sret";
      break;

    case InstId::wfi:
      out << "wfi";
      break;

    case InstId::sfence_vma:
      printRs1Rs2(*this, out, "sfence.vma ", di);
      break;

    case InstId::c_addi4spn:
      printRegImm(*this, out, "c.addi4spn ", di.op0(), di.op2As<int32_t>() >> 2);
      break;

    case InstId::c_fld:
      printFpLdSt(*this, out, "c.fld", di);
      break;

    case InstId::c_lq:
      out << "illegal";
      break;

    case InstId::c_lw:
      printLdSt(*this, out, "c.lw", di);
      break;

    case InstId::c_flw:
      printFpLdSt(*this, out, "c.flw", di);
      break;

    case InstId::c_ld:
      printLdSt(*this, out, "c.ld", di);
      break;

    case InstId::c_fsd:
      printFpLdSt(*this, out, "c.fsd", di);
      break;

    case InstId::c_sq:
      out << "illegal";
      break;

    case InstId::c_sw:
      printLdSt(*this, out, "c.sw", di);
      break;

    case InstId::c_fsw:
      printFpLdSt(*this, out, "c.fsw", di);
      break;

    case InstId::c_sd:
      printLdSt(*this, out, "c.sd", di);
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
      printRegImm(*this, out, "c.lui", di.op0(), di.op1() >> 12);
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
      printBranch2(*this, out, "c.beqz", di);
      break;

    case InstId::c_bnez:
      printBranch2(*this, out, "c.bnez", di);
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

    case InstId::clz:
      printRdRs1(*this, out, "clz", di);
      break;

    case InstId::ctz:
      printRdRs1(*this, out, "ctz", di);
      break;

    case InstId::cpop:
      printRdRs1(*this, out, "cpop", di);
      break;

    case InstId::clzw:
      printRdRs1(*this, out, "clzw", di);
      break;

    case InstId::ctzw:
      printRdRs1(*this, out, "ctzw", di);
      break;

    case InstId::cpopw:
      printRdRs1(*this, out, "cpopw", di);
      break;

    case InstId::min:
      printRdRs1Rs2(*this, out, "min", di);
      break;

    case InstId::max:
      printRdRs1Rs2(*this, out, "max", di);
      break;

    case InstId::minu:
      printRdRs1Rs2(*this, out, "minu", di);
      break;

    case InstId::maxu:
      printRdRs1Rs2(*this, out, "maxu", di);
      break;

    case InstId::sext_b:
      printRdRs1(*this, out, "sext.b", di);
      break;

    case InstId::sext_h:
      printRdRs1(*this, out, "sext.h", di);
      break;

    case InstId::andn:
      printRdRs1Rs2(*this, out, "andn", di);
      break;

    case InstId::orn:
      printRdRs1Rs2(*this, out, "orn", di);
      break;

    case InstId::xnor:
      printRdRs1Rs2(*this, out, "xnor", di);
      break;

    case InstId::rol:
      printRdRs1Rs2(*this, out, "rol", di);
      break;

    case InstId::ror:
      printRdRs1Rs2(*this, out, "ror", di);
      break;

    case InstId::rori:
      printShiftImm(*this, out, "rori", di);
      break;

    case InstId::rolw:
      printRdRs1Rs2(*this, out, "rolw", di);
      break;

    case InstId::rorw:
      printRdRs1Rs2(*this, out, "rorw", di);
      break;

    case InstId::roriw:
      printShiftImm(*this, out, "roriw", di);
      break;

    case InstId::rev8:
      printRdRs1(*this, out, "rev8", di);
      break;

    case InstId::pack:
      printRdRs1Rs2(*this, out, "pack", di);
      break;

    case InstId::packh:
      printRdRs1Rs2(*this, out, "packh", di);
      break;

    case InstId::packu:
      printRdRs1Rs2(*this, out, "packu", di);
      break;

    case InstId::packw:
      printRdRs1Rs2(*this, out, "packw", di);
      break;

    case InstId::packuw:
      printRdRs1Rs2(*this, out, "packuw", di);
      break;

    case InstId::grev:
      printRdRs1Rs2(*this, out, "grev", di);
      break;

    case InstId::grevi:
      printShiftImm(*this, out, "grevi", di);
      break;

    case InstId::grevw:
      printRdRs1Rs2(*this, out, "grevw", di);
      break;

    case InstId::greviw:
      printShiftImm(*this, out, "greviw", di);
      break;

    case InstId::gorc:
      printRdRs1Rs2(*this, out, "gorc", di);
      break;

    case InstId::gorci:
      printShiftImm(*this, out, "gorci", di);
      break;

    case InstId::gorcw:
      printRdRs1Rs2(*this, out, "gorcw", di);
      break;

    case InstId::gorciw:
      printShiftImm(*this, out, "gorciw", di);
      break;

    case InstId::shfl:
      printRdRs1Rs2(*this, out, "shfl", di);
      break;

    case InstId::shflw:
      printRdRs1Rs2(*this, out, "shflw", di);
      break;

    case InstId::shfli:
      printShiftImm(*this, out, "shfli", di);
      break;

    case InstId::unshfl:
      printRdRs1Rs2(*this, out, "unshfl", di);
      break;

    case InstId::unshfli:
      printShiftImm(*this, out, "unshfli", di);
      break;

    case InstId::unshflw:
      printRdRs1Rs2(*this, out, "unshflw", di);
      break;

    case InstId::xperm_n:
      printRdRs1Rs2(*this, out, "xperm_n", di);
      break;

    case InstId::xperm_b:
      printRdRs1Rs2(*this, out, "xperm_b", di);
      break;

    case InstId::xperm_h:
      printRdRs1Rs2(*this, out, "xperm_h", di);
      break;

    case InstId::xperm_w:
      printRdRs1Rs2(*this, out, "xperm_w", di);
      break;

    case InstId::bset:
      printRdRs1Rs2(*this, out, "bset", di);
      break;

    case InstId::bclr:
      printRdRs1Rs2(*this, out, "bclr", di);
      break;

    case InstId::binv:
      printRdRs1Rs2(*this, out, "binv", di);
      break;

    case InstId::bext:
      printRdRs1Rs2(*this, out, "bext", di);
      break;

    case InstId::bseti:
      printShiftImm(*this, out, "bseti", di);
      break;

    case InstId::bclri:
      printShiftImm(*this, out, "bclri", di);
      break;

    case InstId::binvi:
      printShiftImm(*this, out, "binvi", di);
      break;

    case InstId::bexti:
      printShiftImm(*this, out, "bexti", di);
      break;

    case InstId::bcompress:
      printRdRs1Rs2(*this, out, "bcompress", di);
      break;

    case InstId::bdecompress:
      printRdRs1Rs2(*this, out, "bdecompress", di);
      break;

    case InstId::bcompressw:
      printRdRs1Rs2(*this, out, "bcompressw", di);
      break;

    case InstId::bdecompressw:
      printRdRs1Rs2(*this, out, "bdecompressw", di);
      break;

    case InstId::bfp:
      printRdRs1Rs2(*this, out, "bfp", di);
      break;

    case InstId::bfpw:
      printRdRs1Rs2(*this, out, "bfpw", di);
      break;

    case InstId::clmul:
      printRdRs1Rs2(*this, out, "clmul", di);
      break;

    case InstId::clmulh:
      printRdRs1Rs2(*this, out, "clmulh", di);
      break;

    case InstId::clmulr:
      printRdRs1Rs2(*this, out, "clmulr", di);
      break;

    case InstId::sh1add:
      printRdRs1Rs2(*this, out, "sh1add", di);
      break;

    case InstId::sh2add:
      printRdRs1Rs2(*this, out, "sh2add", di);
      break;

    case InstId::sh3add:
      printRdRs1Rs2(*this, out, "sh3add", di);
      break;

    case InstId::sh1add_uw:
      printRdRs1Rs2(*this, out, "sh1add.uw ", di);
      break;

    case InstId::sh2add_uw:
      printRdRs1Rs2(*this, out, "sh2add.uw ", di);
      break;

    case InstId::sh3add_uw:
      printRdRs1Rs2(*this, out, "sh3add.uw ", di);
      break;

    case InstId::add_uw:
      printRdRs1Rs2(*this, out, "add.uw", di);
      break;

    case InstId::slli_uw:
      printShiftImm(*this, out, "slli.uw", di);
      break;

    case InstId::crc32_b:
      printRdRs1(*this, out, "crc32.b", di);
      break;

    case InstId::crc32_h:
      printRdRs1(*this, out, "crc32.h", di);
      break;

    case InstId::crc32_w:
      printRdRs1(*this, out, "crc32.w", di);
      break;

    case InstId::crc32_d:
      printRdRs1(*this, out, "crc32.d", di);
      break;

    case InstId::crc32c_b:
      printRdRs1(*this, out, "crc32c.b", di);
      break;

    case InstId::crc32c_h:
      printRdRs1(*this, out, "crc32c.h", di);
      break;

    case InstId::crc32c_w:
      printRdRs1(*this, out, "crc32c.w", di);
      break;

    case InstId::crc32c_d:
      printRdRs1(*this, out, "crc32c.d", di);
      break;

    case InstId::bmator:
      printRdRs1Rs2(*this, out, "bmator", di);
      break;

    case InstId::bmatxor:
      printRdRs1Rs2(*this, out, "bmatxor", di);
      break;

    case InstId::bmatflip:
      printRdRs1(*this, out, "bmatflip", di);
      break;

    case InstId::cmov:
      printRdRs2Rs1Rs3(*this, out, "cmov", di);
      break;

    case InstId::cmix:
      printRdRs2Rs1Rs3(*this, out, "cmix", di);
      break;

    case InstId::fsl:
      printRdRs1Rs3Rs2(*this, out, "fsl", di);
      break;

    case InstId::fsr:
      printRdRs1Rs3Rs2(*this, out, "fsr", di);
      break;

    case InstId::fsri:
      printRdRs1Rs3Imm(*this, out, "fsri", di);
      break;

    case InstId::fslw:
      printRdRs1Rs3Rs2(*this, out, "fslw", di);
      break;

    case InstId::fsrw:
      printRdRs1Rs3Rs2(*this, out, "fsrw", di);
      break;

    case InstId::fsriw:
      printRdRs1Rs3Imm(*this, out, "fsriw", di);
      break;

    case InstId::load64:
      printLdSt(*this, out, "load64", di);
      break;

    case InstId::store64:
      printLdSt(*this, out, "store64", di);
      break;

    case InstId::bbarrier:
      out << "bbarrier";
      break;

    case InstId::vsetvli:
      {
        out << "vsetvli x" << di.op0() << ", x" << di.op1() << ", ";
        std::string mm = ((di.op2() >> 7) & 1) ? "ma" : "mu";
        std::string tt = ((di.op2() >> 6) & 1) ? "ta" : "tu";
        auto gm = VecRegs::to_string(GroupMultiplier(di.op2() & 7));
        auto ew = VecRegs::to_string(ElementWidth((di.op2() >> 3) & 7));
        out << ew << ',' << gm << ',' << tt << ',' << mm;
      }
      break;

    case InstId::vsetvl:
      out << "vsetvl x" << di.op0() << ", x" << di.op1() << ", x" << di.op2();
      break;

    case InstId::vadd_vv:
      printVec_vv(*this, out, "vadd.vv", di);
      break;

    case InstId::vadd_vx:
      printVec_vx(*this, out, "vadd.vx", di);
      break;

    case InstId::vadd_vi:
      printVec_vi(*this, out, "vadd.vi", di);
      break;

    case InstId::vsub_vv:
      printVec_vv(*this, out, "sub.vv", di);
      break;

    case InstId::vsub_vx:
      printVec_vx(*this, out, "vsub.vx", di);
      break;

    case InstId::vrsub_vx:
      printVec_vx(*this, out, "vrsub.vx", di);
      break;

    case InstId::vrsub_vi:
      printVec_vi(*this, out, "vrsub.vi", di);
      break;

    case InstId::vwaddu_vv:
      printVec_vv(*this, out, "vwaddu.vv", di, false);
      break;

    case InstId::vwaddu_vx:
      printVec_vx(*this, out, "vwaddu.vx", di, false);
      break;

    case InstId::vwsubu_vv:
      printVec_vv(*this, out, "vwsubu.vv", di, false);
      break;

    case InstId::vwsubu_vx:
      printVec_vx(*this, out, "vwsubu.vx", di, false);
      break;

    case InstId::vwadd_vv:
      printVec_vv(*this, out, "vwadd.vv", di, false);
      break;

    case InstId::vwadd_vx:
      printVec_vx(*this, out, "vwadd.vx", di, false);
      break;

    case InstId::vwsub_vv:
      printVec_vv(*this, out, "vwsub.vv", di, false);
      break;

    case InstId::vwsub_vx:
      printVec_vx(*this, out, "vwsub.vx", di, false);
      break;

    case InstId::vwaddu_wv:
      printVec_vv(*this, out, "vwaddu.wv", di, false);
      break;

    case InstId::vwaddu_wx:
      printVec_vx(*this, out, "vwaddu.wx", di, false);
      break;

    case InstId::vwsubu_wv:
      printVec_vv(*this, out, "vwsubu.wv", di, false);
      break;

    case InstId::vwsubu_wx:
      printVec_vx(*this, out, "vwsubu.wx", di, false);
      break;

    case InstId::vwadd_wv:
      printVec_vv(*this, out, "vwadd.wv", di, false);
      break;

    case InstId::vwadd_wx:
      printVec_vx(*this, out, "vwadd.wx", di, false);
      break;

    case InstId::vwsub_wv:
      printVec_vv(*this, out, "vwsub.wv", di, false);
      break;

    case InstId::vwsub_wx:
      printVec_vx(*this, out, "vwsub.wx", di, false);
      break;

    case InstId::vmseq_vv:
      printVec_vv(*this, out, "vmseq.vv", di);
      break;

    case InstId::vmseq_vx:
      printVec_vx(*this, out, "vmseq.vx", di);
      break;

    case InstId::vmseq_vi:
      printVec_vi(*this, out, "vmseq.vi", di);
      break;

    case InstId::vmsne_vv:
      printVec_vv(*this, out, "vmsne.vv", di);
      break;

    case InstId::vmsne_vx:
      printVec_vx(*this, out, "vmsne.vx", di);
      break;

    case InstId::vmsne_vi:
      printVec_vi(*this, out, "vmsne.vi", di);
      break;

    case InstId::vmsltu_vv:
      printVec_vv(*this, out, "vmsltu.vv", di);
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vmsltu_vx:
      printVec_vx(*this, out, "vmsltu.vx", di);
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vmslt_vv:
      printVec_vv(*this, out, "vmslt.vv", di);
      break;

    case InstId::vmslt_vx:
      printVec_vx(*this, out, "vmslt.vx", di);
      break;

    case InstId::vmsleu_vv:
      printVec_vv(*this, out, "vmsleu.vv", di);
      break;

    case InstId::vmsleu_vx:
      printVec_vx(*this, out, "vmsleu.vx", di);
      break;

    case InstId::vmsleu_vi:
      printVec_vi(*this, out, "vmsleu.vi", di);
      break;

    case InstId::vmsle_vv:
      printVec_vv(*this, out, "vmsle.vv", di);
      break;

    case InstId::vmsle_vx:
      printVec_vx(*this, out, "vmsle.vx", di);
      break;

    case InstId::vmsle_vi:
      printVec_vi(*this, out, "vmsle.vi", di);
      break;

    case InstId::vmsgtu_vx:
      printVec_vx(*this, out, "vmsgtu.vx", di);
      break;

    case InstId::vmsgtu_vi:
      printVec_vi(*this, out, "vmsgtu.vi", di);
      break;

    case InstId::vmsgt_vx:
      printVec_vx(*this, out, "vmsgt.vx", di);
      break;

    case InstId::vmsgt_vi:
      printVec_vi(*this, out, "vmsgt.vi", di);
      break;

    case InstId::vminu_vv:
      printVec_vv(*this, out, "vminu.vv", di);
      break;

    case InstId::vminu_vx:
      printVec_vx(*this, out, "vminu.vx", di);
      break;

    case InstId::vmin_vv:
      printVec_vv(*this, out, "vmin.vv", di);
      break;

    case InstId::vmin_vx:
      printVec_vx(*this, out, "vmin.vx", di);
      break;

    case InstId::vmaxu_vv:
      printVec_vv(*this, out, "vmaxu.vv", di);
      break;

    case InstId::vmaxu_vx:
      printVec_vx(*this, out, "vmaxu.vx", di);
      break;

    case InstId::vmax_vv:
      printVec_vv(*this, out, "vmax.vv", di);
      break;

    case InstId::vmax_vx:
      printVec_vx(*this, out, "vmax.vx", di);
      break;

    case InstId::vand_vv:
      printVec_vv(*this, out, "vand.vv", di);
      break;

    case InstId::vand_vx:
      printVec_vx(*this, out, "vand.vx", di);
      break;

    case InstId::vand_vi:
      printVec_vi(*this, out, "vand.vi", di);
      break;

    case InstId::vor_vv:
      printVec_vv(*this, out, "vor.vv", di);
      break;

    case InstId::vor_vx:
      printVec_vx(*this, out, "vor.vx", di);
      break;

    case InstId::vor_vi:
      printVec_vi(*this, out, "vor.vi", di);
      break;

    case InstId::vxor_vv:
      printVec_vv(*this, out, "vxor.vv", di);
      break;

    case InstId::vxor_vx:
      printVec_vx(*this, out, "vxor.vx", di);
      break;

    case InstId::vxor_vi:
      printVec_vi(*this, out, "vxor.vi", di);
      break;

    case InstId::vsll_vv:
      printVec_vv(*this, out, "vsll.vv", di);
      break;

    case InstId::vsll_vx:
      printVec_vx(*this, out, "vsll.vx", di);
      break;

    case InstId::vsll_vi:
      printVec_vi(*this, out, "vsll.vi", di);
      break;

    case InstId::vsrl_vv:
      printVec_vv(*this, out, "xsrl.vv", di);
      break;

    case InstId::vsrl_vx:
      printVec_vx(*this, out, "xsrl.vx", di);
      break;

    case InstId::vsrl_vi:
      printVec_vi(*this, out, "xsrl.vi", di);
      break;

    case InstId::vsra_vv:
      printVec_vv(*this, out, "vsra.vv", di);
      break;

    case InstId::vsra_vx:
      printVec_vx(*this, out, "vsra.vx", di);
      break;

    case InstId::vsra_vi:
      printVec_vi(*this, out, "vsra.vi", di);
      break;

    case InstId::vnsrl_wv:
      printVec_vv(*this, out, "vnsrl.wv", di);
      break;

    case InstId::vnsrl_wx:
      printVec_vx(*this, out, "vnsrl.wx", di);
      break;

    case InstId::vnsrl_wi:
      printVec_vi(*this, out, "vnsrl.wi", di);
      break;

    case InstId::vnsra_wv:
      printVec_vv(*this, out, "vnsra.wv", di);
      break;

    case InstId::vnsra_wx:
      printVec_vx(*this, out, "vnsra.wx", di);
      break;

    case InstId::vnsra_wi:
      printVec_vi(*this, out, "vnsra.wi", di);
      break;

    case InstId::vrgather_vv:
      printVec_vv(*this, out, "vrgather.vv", di);
      break;

    case InstId::vrgather_vx:
      printVec_vx(*this, out, "vrgather.vx", di);
      break;

    case InstId::vrgather_vi:
      printVec_vi(*this, out, "vrgather.vi", di);
      break;

    case InstId::vrgatherei16_vv:
      printVec_vv(*this, out, "vrgatherei16.vv", di);
      break;

    case InstId::vcompress_vm:
      printVec_vv(*this, out, "vwcompress.vv", di, false);
      break;

    case InstId::vredsum_vs:
      printVec_vv(*this, out, "vredsum.vs", di);
      break;

    case InstId::vredand_vs:
      printVec_vv(*this, out, "vredand.vs", di);
      break;

    case InstId::vredor_vs:
      printVec_vv(*this, out, "vredor.vs", di);
      break;

    case InstId::vredxor_vs:
      printVec_vv(*this, out, "vredxor.vs", di);
      break;

    case InstId::vredminu_vs:
      printVec_vv(*this, out, "vredminu.vs", di);
      break;

    case InstId::vredmin_vs:
      printVec_vv(*this, out, "vredmin.vs", di);
      break;

    case InstId::vredmaxu_vs:
      printVec_vv(*this, out, "vredmaxu.vs", di);
      break;

    case InstId::vredmax_vs:
      printVec_vv(*this, out, "vredmax.vs", di);
      break;

    case InstId::vmand_mm:
      printVec_vv(*this, out, "vmand.mm", di, false);
      break;

    case InstId::vmnand_mm:
      printVec_vv(*this, out, "vmnand.mm", di, false);
      break;

    case InstId::vmandnot_mm:
      printVec_vv(*this, out, "vmandnot.mm", di, false);
      break;

    case InstId::vmxor_mm:
      printVec_vv(*this, out, "vmxor.mm", di, false);
      break;

    case InstId::vmor_mm:
      printVec_vv(*this, out, "vmor.mm", di, false);
      break;

    case InstId::vmnor_mm:
      printVec_vv(*this, out, "vmnot.mm", di, false);
      break;

    case InstId::vmornot_mm:
      printVec_vv(*this, out, "vmornot.mm", di, false);
      break;

    case InstId::vmxnor_mm:
      printVec_vv(*this, out, "vmxnor.mm", di, false);
      break;

    case InstId::vpopc_m:
      out << "vpopc.m " << intRegName(di.op0()) << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vfirst_m:
      out << "vpopc.m " << intRegName(di.op0()) << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vmsbf_m:
      out << "vmsbf.m v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vmsif_m:
      out << "vmsif.m v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vmsof_m:
      out << "vmsof.m v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::viota_m:
      out << "viota.m v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vid_v:
      out << "vid.v v" << di.op0();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vslideup_vx:
      printVec_vx(*this, out, "vslideup.vx", di);
      break;

    case InstId::vslideup_vi:
      printVec_vi(*this, out, "vslideup.vi", di);
      break;

    case InstId::vslide1up_vx:
      printVec_vx(*this, out, "vslide1up.vx", di);
      break;

    case InstId::vslidedown_vx:
      printVec_vx(*this, out, "vslidedown.vx", di);
      break;

    case InstId::vslidedown_vi:
      printVec_vi(*this, out, "vslidedown.vi", di);
      break;

    case InstId::vslide1down_vx:
      printVec_vx(*this, out, "vslide1doen.vx", di);
      break;

    case InstId::vmul_vv:
      printVec_vv(*this, out, "vmul.vv", di);
      break;

    case InstId::vmul_vx:
      printVec_vx(*this, out, "vmul.vx", di);
      break;

    case InstId::vmulh_vv:
      printVec_vv(*this, out, "vmulh.vv", di);
      break;

    case InstId::vmulh_vx:
      printVec_vx(*this, out, "vmulx.vx", di);
      break;

    case InstId::vmulhu_vv:
      printVec_vv(*this, out, "vmulhu.vv", di);
      break;

    case InstId::vmulhu_vx:
      printVec_vv(*this, out, "vmulhu.vx", di);
      break;

    case InstId::vmulhsu_vv:
      printVec_vv(*this, out, "vmulhsu.vv", di);
      break;

    case InstId::vmulhsu_vx:
      printVec_vx(*this, out, "vmulhsu.vx", di);
      break;

    case InstId::vmadd_vv:
      printVec_vv(*this, out, "vmadd.vv", di);
      break;

    case InstId::vmadd_vx:
      printVec_xv(*this, out, "vmadd.vx", di);
      break;

    case InstId::vnmsub_vv:
      printVec_vv(*this, out, "vnmsub.vv", di);
      break;

    case InstId::vnmsub_vx:
      printVec_xv(*this, out, "vnmsub.vx", di);
      break;

    case InstId::vmacc_vv:
      printVec_vv(*this, out, "vmacc.vv", di);
      break;

    case InstId::vmacc_vx:
      printVec_xv(*this, out, "vmacc.vx", di);
      break;

    case InstId::vnmsac_vv:
      printVec_vv(*this, out, "vnmsac.vv", di);
      break;

    case InstId::vnmsac_vx:
      printVec_xv(*this, out, "vnmsac.vx", di);
      break;

    case InstId::vwmulu_vv:
      printVec_vv(*this, out, "vwmulu.vv", di);
      break;

    case InstId::vwmulu_vx:
      printVec_vx(*this, out, "vwmulu.vx", di);
      break;

    case InstId::vwmul_vv:
      printVec_vv(*this, out, "vwmul.vv", di);
      break;

    case InstId::vwmul_vx:
      printVec_vx(*this, out, "vwmul.vx", di);
      break;

    case InstId::vwmulsu_vv:
      printVec_vv(*this, out, "vwmulsu.vv", di);
      break;

    case InstId::vwmulsu_vx:
      printVec_vx(*this, out, "vwmulsu.vx", di);
      break;

    case InstId::vwmaccu_vv:
      printVec_vv(*this, out, "vwmaccu.vv", di);
      break;

    case InstId::vwmaccu_vx:
      printVec_vx(*this, out, "vwmaccu.vx", di);
      break;

    case InstId::vwmacc_vv:
      printVec_vv(*this, out, "vmacc.vv", di);
      break;

    case InstId::vwmacc_vx:
      printVec_vx(*this, out, "vmacc.vx", di);
      break;

    case InstId::vwmaccsu_vv:
      printVec_vv(*this, out, "vwmaccsu.vv", di);
      break;

    case InstId::vwmaccsu_vx:
      printVec_vx(*this, out, "vwmaccsu.vx", di);
      break;

    case InstId::vwmaccus_vx:
      printVec_vx(*this, out, "vwmaccus.vx", di);
      break;

    case InstId::vdivu_vv:
      printVec_vv(*this, out, "vdivu.vv", di);
      break;

    case InstId::vdivu_vx:
      printVec_vx(*this, out, "vdivu.vx", di);
      break;

    case InstId::vdiv_vv:
      printVec_vv(*this, out, "vdiv.vv", di);
      break;

    case InstId::vdiv_vx:
      printVec_vx(*this, out, "vdiv.vx", di);
      break;

    case InstId::vremu_vv:
      printVec_vv(*this, out, "vremu.vv", di);
      break;

    case InstId::vremu_vx:
      printVec_vx(*this, out, "vremu.vx", di);
      break;

    case InstId::vrem_vv:
      printVec_vv(*this, out, "vrem.vv", di);
      break;

    case InstId::vrem_vx:
      printVec_vx(*this, out, "vrem.vx", di);
      break;

    case InstId::vsext_vf2:
      out << "vsext.vf2 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vsext_vf4:
      out << "vsext.vf4 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vsext_vf8:
      out << "vsext.vf8 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vzext_vf2:
      out << "vzext.vf2 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vzext_vf4:
      out << "vzext.vf4 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vzext_vf8:
      out << "vzext.vf8 v" << di.op0() << ", v" << di.op1();
      if (di.isMasked()) out << ", v0.t";
      break;

    case InstId::vadc_vvm:
      out << "vadc.vvm v" << di.op0() << ", v" << di.op1() << ", v" << di.op2();
      out << ", v0";
      break;

    case InstId::vadc_vxm:
      out << "vadc.vxm v" << di.op0() << ", v" << di.op1() << ", x" << di.op2();
      out << ", v0";
      break;

    case InstId::vadc_vim:
      out << "vadc.vim v" << di.op0() << ", v" << di.op1() << ", " << di.op2As<int32_t>();
      out << ", v0";
      break;

    case InstId::vsbc_vvm:
      out << "vsbc.vvm v" << di.op0() << ", v" << di.op1() << ", v" << di.op2();
      out << ", v0";
      break;

    case InstId::vsbc_vxm:
      out << "vsbc.vxm v" << di.op0() << ", v" << di.op1() << ", x" << di.op2();
      out << ", v0";
      break;

    case InstId::vmadc_vvm:
      out << "vmadc.vvm v" << di.op0() << ", v" << di.op1() << ", " << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vmadc_vxm:
      out << "vmadc.vxm v" << di.op0() << ", v" << di.op1() << ", x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vmadc_vim:
      out << "vmadc.vim v" << di.op0() << ", v" << di.op1() << ", " << di.op2As<int32_t>();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vmsbc_vvm:
      out << "vmsbc.vvm v" << di.op0() << ", v" << di.op1() << ", v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vmsbc_vxm:
      out << "vmsbc.vxm v" << di.op0() << ", v" << di.op1() << ", x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vmerge_vv:
      printVec_vv(*this, out, "vmerge.vv", di, false);
      break;

    case InstId::vmerge_vx:
      printVec_vx(*this, out, "vmerge.vx", di, false);
      break;

    case InstId::vmerge_vi:
      printVec_vi(*this, out, "vmerge.vi", di);
      break;

    case InstId::vmv_x_s:
      out << "vmv.x.s x" << intRegName(di.op0()) << ", v" << di.op1();
      break;

    case InstId::vmv_s_x:
      out << "vmv.s.x v" << di.op0() << ", " << intRegName(di.op1());
      break;

    case InstId::vmv_v_v:
      out << "vmv.v.v v" << di.op0() << ", v" << di.op1();
      break;

    case InstId::vmv_v_x:
      out << "vmv.v.x v" << di.op0() << ", " << intRegName(di.op1());
      break;

    case InstId::vmv_v_i:
      out << "vmv.v.i v" << di.op0() << ", " << di.op1As<int32_t>();
      break;

    case InstId::vmv1r_v:
      out << "vmv1r.v v" << di.op0() << ", v" << di.op1();
      break;

    case InstId::vmv2r_v:
      out << "vmv2r.v v" << di.op0() << ", v" << di.op1();
      break;

    case InstId::vmv4r_v:
      out << "vmv4r.v v" << di.op0() << ", v" << di.op1();
      break;

    case InstId::vmv8r_v:
      out << "vmv8r.v v" << di.op0() << ", v" << di.op1();
      break;

    case InstId::vsaddu_vv:
      printVec_vv(*this, out, "vsaddu.vv", di);
      break;

    case InstId::vsaddu_vx:
      printVec_vx(*this, out, "vsaddu.vx", di);
      break;

    case InstId::vsaddu_vi:
      printVec_vi(*this, out, "vsaddu.vi", di);
      break;

    case InstId::vsadd_vv:
      printVec_vv(*this, out, "vsadd.vv", di);
      break;

    case InstId::vsadd_vx:
      printVec_vx(*this, out, "vsadd.vx", di);
      break;

    case InstId::vsadd_vi:
      printVec_vi(*this, out, "vsadd.vi", di);
      break;

    case InstId::vssubu_vv:
      printVec_vv(*this, out, "vssubu.vv", di);
      break;

    case InstId::vssubu_vx:
      printVec_vx(*this, out, "vssubu.vx", di);
      break;

    case InstId::vssub_vv:
      printVec_vv(*this, out, "vssub.vv", di);
      break;

    case InstId::vssub_vx:
      printVec_vx(*this, out, "vssub.vx", di);
      break;

    case InstId::vaaddu_vv:
      printVec_vv(*this, out, "vaaddu.vv", di);
      break;

    case InstId::vaaddu_vx:
      printVec_vx(*this, out, "vaaddu.vx", di);
      break;

    case InstId::vaadd_vv:
      printVec_vv(*this, out, "vaadd.vv", di);
      break;

    case InstId::vaadd_vx:
      printVec_vx(*this, out, "vaadd.vx", di);
      break;

    case InstId::vasubu_vv:
      printVec_vv(*this, out, "vasubu.vv", di);
      break;

    case InstId::vasubu_vx:
      printVec_vx(*this, out, "vasubu.vx", di);
      break;

    case InstId::vasub_vv:
      printVec_vv(*this, out, "vasub.vv", di);
      break;

    case InstId::vasub_vx:
      printVec_vx(*this, out, "vasub.vx", di);
      break;

    case InstId::vsmul_vv:
      printVec_vv(*this, out, "vsmul.vv", di);
      break;

    case InstId::vsmul_vx:
      printVec_vx(*this, out, "vsmul.vx", di);
      break;

    case InstId::vssrl_vv:
      printVec_vv(*this, out, "vssrl.vv", di);
      break;

    case InstId::vssrl_vx:
      printVec_vx(*this, out, "vssrl.vx", di);
      break;

    case InstId::vssrl_vi:
      printVec_vi(*this, out, "vssrl.vi", di);
      break;

    case InstId::vssra_vv:
      printVec_vv(*this, out, "vssra.vv", di);
      break;

    case InstId::vssra_vx:
      printVec_vx(*this, out, "vssra.vx", di);
      break;

    case InstId::vssra_vi:
      printVec_vi(*this, out, "vssra.vi", di);
      break;

    case InstId::vnclipu_wv:
      printVec_vv(*this, out, "vnclipu.wv", di);
      break;

    case InstId::vnclipu_wx:
      printVec_vx(*this, out, "vnclipu.wx", di);
      break;

    case InstId::vnclipu_wi:
      printVec_vi(*this, out, "vnclipu.wi", di);
      break;

    case InstId::vnclip_wv:
      printVec_vv(*this, out, "vnclip.wv", di);
      break;

    case InstId::vnclip_wx:
      printVec_vx(*this, out, "vnclip.vx", di);
      break;

    case InstId::vnclip_wi:
      printVec_vi(*this, out, "vnclip.wi", di);
      break;

    case InstId::vle8_v:
      out << "vle8.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle16_v:
      out << "vle16.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle32_v:
      out << "vle32.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle64_v:
      out << "vle64.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle128_v:
      out << "vle128.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle256_v:
      out << "vle256.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle512_v:
      out << "vle512.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle1024_v:
      out << "vle1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse8_v:
      out << "vse8.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse16_v:
      out << "vse16.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse32_v:
      out << "vse32.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse64_v:
      out << "vse64.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse128_v:
      out << "vse128.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse256_v:
      out << "vse256.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse512_v:
      out << "vse512.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vse1024_v:
      out << "vse1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre8_v:
      out << "vl" << di.op3() << "re8.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre16_v:
      out << "vl" << di.op3() << "re16.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre32_v:
      out << "vl" << di.op3() << "re32.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre64_v:
      out << "vl" << di.op3() << "re64.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre128_v:
      out << "vl" << di.op3() << "re128.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre256_v:
      out << "vl" << di.op3() << "re256.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre512_v:
      out << "vl" << di.op3() << "re512.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlre1024_v:
      out << "vl" << di.op3() << "re1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre8_v:
      out << "vs" << di.op3() << "re8.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre16_v:
      out << "vs" << di.op3() << "re16.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre32_v:
      out << "vs" << di.op3() << "re32.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre64_v:
      out << "vs" << di.op3() << "re64.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre128_v:
      out << "vs" << di.op3() << "re128.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre256_v:
      out << "vs" << di.op3() << "re256.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre512_v:
      out << "vs" << di.op3() << "re512.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsre1024_v:
      out << "vs" << di.op3() << "re1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle8ff_v:
      out << "vle8ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle16ff_v:
      out << "vle16ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle32ff_v:
      out << "vle32ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle64ff_v:
      out << "vle64ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle128ff_v:
      out << "vle128ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle256ff_v:
      out << "vle256ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle512ff_v:
      out << "vle512ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vle1024ff_v:
      out << "vle1024ff.v v" << di.op0() << ", (" << intRegName(di.op1()) << ")";
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse8_v:
      out << "vlse8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse16_v:
      out << "vlse16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse32_v:
      out << "vlse32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse64_v:
      out << "vlse64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse128_v:
      out << "vlse128.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse256_v:
      out << "vlse256.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse512_v:
      out << "vlse512.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlse1024_v:
      out << "vlse1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse8_v:
      out << "vsse8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse16_v:
      out << "vsse16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse32_v:
      out << "vsse32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse64_v:
      out << "vsse64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse128_v:
      out << "vsse128.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse256_v:
      out << "vsse256.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse512_v:
      out << "vsse512.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsse1024_v:
      out << "vsse1024.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), x" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlxei8_v:
      out << "vlxei8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlxei16_v:
      out << "vlxei16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlxei32_v:
      out << "vlxei32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vlxei64_v:
      out << "vlxei64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vluxei8_v:
      out << "vluxei8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vluxei16_v:
      out << "vluxei16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vluxei32_v:
      out << "vluxei32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vluxei64_v:
      out << "vluxei64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsxei8_v:
      out << "vsxei8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsxei16_v:
      out << "vsxei16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsxei32_v:
      out << "vsxei32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsxei64_v:
      out << "vsxei64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsuxei8_v:
      out << "vsuxei8.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsuxei16_v:
      out << "vsuxei16.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsuxei32_v:
      out << "vsuxei32.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vsuxei64_v:
      out << "vsuxei64.v v" << di.op0() << ", (" << intRegName(di.op1()) << "), v" << di.op2();
      if (di.isMasked()) out << ", v0";
      break;

    case InstId::vfadd_vv:
      printVec_vv(*this, out, "vfadd.vv", di);
      break;

    case InstId::vfadd_vf:
      printVec_vf(*this, out, "vfadd.vf", di);
      break;

    case InstId::vfsub_vv:
      printVec_vv(*this, out, "vfsub.vv", di);
      break;

    case InstId::vfsub_vf:
      printVec_vf(*this, out, "vfsub.vf", di);
      break;

    case InstId::vfrsub_vf:
      printVec_vf(*this, out, "vfrsub.vf", di);
      break;

    case InstId::vfwadd_vv:
      printVec_vv(*this, out, "vfwadd.vv", di);
      break;

    case InstId::vfwadd_vf:
      printVec_vf(*this, out, "vfwadd.vf", di);
      break;

    case InstId::vfwsub_vv:
      printVec_vv(*this, out, "vfwsub.vv", di);
      break;

    case InstId::vfwsub_vf:
      printVec_vf(*this, out, "vfwsub.vf", di);
      break;

    case InstId::vfwadd_wv:
      printVec_vv(*this, out, "vfwadd.wv", di);
      break;

    case InstId::vfwadd_wf:
      printVec_vf(*this, out, "vfwadd.wf", di);
      break;

    case InstId::vfwsub_wv:
      printVec_vv(*this, out, "vfwsub.wv", di);
      break;

    case InstId::vfwsub_wf:
      printVec_vf(*this, out, "vfwsub.wf", di);
      break;

    case InstId::vfmul_vv:
      printVec_vv(*this, out, "vfmul.vv", di);
      break;

    case InstId::vfmul_vf:
      printVec_vf(*this, out, "vfmul.vf", di);
      break;

    case InstId::vfdiv_vv:
      printVec_vv(*this, out, "vfdiv.vv", di);
      break;

    case InstId::vfdiv_vf:
      printVec_vf(*this, out, "vfdiv.vf", di);
      break;

    case InstId::vfrdiv_vf:
      printVec_vf(*this, out, "vfrdiv.vf", di);
      break;

    case InstId::vfmadd_vv:
      printVec_vv(*this, out, "vfmadd.vv", di);
      break;

    case InstId::vfmadd_vf:
      printVec_fv(*this, out, "vfmadd.vf", di);
      break;

    case InstId::vfnmadd_vv:
      printVec_vv(*this, out, "vfnmadd.vv", di);
      break;

    case InstId::vfnmadd_vf:
      printVec_fv(*this, out, "vfnmadd.vf", di);
      break;

    case InstId::vfmsub_vv:
      printVec_vv(*this, out, "vfmsub.vv", di);
      break;

    case InstId::vfmsub_vf:
      printVec_fv(*this, out, "vfmsub.vf", di);
      break;

    case InstId::vfnmsub_vv:
      printVec_vv(*this, out, "vfnmsub.vv", di);
      break;

    case InstId::vfnmsub_vf:
      printVec_fv(*this, out, "vfnmsub.vf", di);
      break;

    case InstId::vfmacc_vv:
      printVec_vv(*this, out, "vfmacc.vv", di);
      break;

    case InstId::vfmacc_vf:
      printVec_fv(*this, out, "vfmacc.vf", di);
      break;

    case InstId::vfnmacc_vv:
      printVec_vv(*this, out, "vfnmacc.vv", di);
      break;

    case InstId::vfnmacc_vf:
      printVec_fv(*this, out, "vfnmacc.vf", di);
      break;

    case InstId::vfmsac_vv:
      printVec_vv(*this, out, "vfmsac.vv", di);
      break;

    case InstId::vfmsac_vf:
      printVec_fv(*this, out, "vfmsac.vf", di);
      break;

    case InstId::vfnmsac_vv:
      printVec_vv(*this, out, "vfnmsac.vv", di);
      break;

    case InstId::vfnmsac_vf:
      printVec_fv(*this, out, "vfnmsac.vf", di);
      break;

    default:
      out << "illegal";
    }
}


template <typename URV>
void
Hart<URV>::disassembleInst(const DecodedInst& di, std::string& str)
{
  str.clear();

  std::ostringstream oss;
  disassembleInst(di, oss);
  str = oss.str();
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
