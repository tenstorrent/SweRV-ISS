#include "DecodedInst.hpp"
#include "util.hpp"


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


static
std::string
insertFieldCountInName(std::string_view name, unsigned count, unsigned n)
{
  std::string res = util::join("", name.substr(0, n), std::to_string(count), name.substr(n));
  return res;
}


std::string_view
DecodedInst::name() const
{
  if (entry_)
    {
      auto id = entry_->instId();
      auto name = entry_->name();
      if (id >= InstId::vlre8_v and id <= InstId::vlre1024_v)
        name = insertFieldCountInName(name, vecFieldCount(), 2);
      else if ((id >= InstId::vlsege8_v and id <= InstId::vssege1024_v) or
               (id >= InstId::vlsege8ff_v and id <= InstId::vlsege1024ff_v))
        name = insertFieldCountInName(name, vecFieldCount(), 5);
      else if (id >= InstId::vlssege8_v and id <= InstId::vsssege1024_v)
        name = insertFieldCountInName(name, vecFieldCount(), 6);
      else if (id >= InstId::vluxsegei8_v and id <= InstId::vsoxsegei1024_v)
        name = insertFieldCountInName(name, vecFieldCount(), 7);
      return name;
    }
  else
    return "illegal";
}
