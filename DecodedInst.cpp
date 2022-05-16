#include "DecodedInst.hpp"


using namespace WdRiscv;


uint32_t
DecodedInst::ithOperand(unsigned i) const
{
  if (i == 0) return op0();
  if (i == 1) return op1();
  if (i == 2) return op2();
  if (i == 3) return op3();
  return 0;
}


int32_t
DecodedInst::ithOperandAsInt(unsigned i) const
{
  return ithOperand(i);
}


void
DecodedInst::setIthOperandValue(unsigned i, uint64_t value)
{
  OperandType type = ithOperandType(i);
  switch(type)
    {
    case OperandType::IntReg:
    case OperandType::FpReg:
    case OperandType::CsReg:
    case OperandType::VecReg:
      if (i < sizeof(values_))
	values_[i] = value;
      break;
      
    case OperandType::Imm:
      break;

    case OperandType::None:
      break;
    }
}



