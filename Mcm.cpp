#include "Mcm.hpp"
#include "System.hpp"

using namespace TTMcm;
using namespace WdRiscv;

template <typename URV>
Mcm<URV>::Mcm(System<URV>& system)
  : system_(system)
{
}


template <typename URV>
Mcm<URV>::~Mcm()
{
}


template class TTMcm::Mcm<uint32_t>;
template class TTMcm::Mcm<uint64_t>;
