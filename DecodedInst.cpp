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
  return static_cast<int32_t>(ithOperand(i));
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


std::string
DecodedInst::name() const
{
  if (entry_)
    {
      auto id   = instId();
      auto name = std::string(entry_->name());

      if (isVector())
	{
	  auto fields = vecFieldCount();
	  if (fields)
	    {
	      // A name like vlre8_v becomes vlrXe8_v where X is the field count.
	      if (id >= InstId::vlre8_v and id <= InstId::vlre1024_v)
		name = insertFieldCountInName(name, fields, 2);
	      else if ((id >= InstId::vlsege8_v and id <= InstId::vssege1024_v) or
		       (id >= InstId::vlsege8ff_v and id <= InstId::vlsege1024ff_v))
		name = insertFieldCountInName(name, fields, 5);
	      else if (id >= InstId::vlssege8_v and id <= InstId::vsssege1024_v)
		name = insertFieldCountInName(name, fields, 6);
	      else if (id >= InstId::vluxsegei8_v and id <= InstId::vsoxsegei1024_v)
		name = insertFieldCountInName(name, fields, 7);
	    }
	  else if (id >= InstId::vmadc_vvm and id <= InstId::vmsbc_vxm and not isMasked())
	    name = name.substr(0, name.size() - 1);  // Remove trailing 'm' if not masked.
	}

      return name;
    }

  return "illegal";
}
