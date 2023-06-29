#include <array>
#include <iostream>
#include <cstdint>
#include "crypto-util.hpp"
#include "DecodedInst.hpp"
#include "Hart.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execAes32dsi(const DecodedInst* di)
{
  if (not isRvzknd() or isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3() * 8;
  uint8_t si = intRegs_.read(di->op2()) >> shamt;
  URV so = aes_sbox_inv(si);
  URV rolSo = (so << shamt) | (so >> (32 - shamt));
  URV result = intRegs_.read(di->op1()) ^ rolSo;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes32dsmi(const DecodedInst* di)
{
  if (not isRvzknd() or isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3() * 8;
  uint8_t si = intRegs_.read(di->op2()) >> shamt;
  uint8_t so = aes_sbox_inv(si);
  URV mixed = aes_mixcolumn_byte_inv(so);
  URV rolMixed = (mixed << shamt) | (mixed >> (32 - shamt));
  URV result = intRegs_.read(di->op1()) ^ rolMixed;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes32esi(const DecodedInst* di)
{
  if (not isRvzkne() or isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3() * 8;
  uint8_t si = intRegs_.read(di->op2()) >> shamt;
  URV so = aes_sbox_fwd(si);
  URV rolSo = (so << shamt) | (so >> (32 - shamt));
  URV result = intRegs_.read(di->op1()) ^ rolSo;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes32esmi(const DecodedInst* di)
{
  if (not isRvzkne() or isRv64())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3() * 8;
  uint8_t si = intRegs_.read(di->op2()) >> shamt;
  uint8_t so = aes_sbox_fwd(si);
  URV mixed = aes_mixcolumn_byte_fwd(so);
  URV rolMixed = (mixed << shamt) | (mixed >> (32 - shamt));
  URV result = intRegs_.read(di->op1()) ^ rolMixed;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64ds(const DecodedInst* di)
{
  if (not isRvzknd() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  URV r1 = intRegs_.read(di->op1());
  URV r2 = intRegs_.read(di->op2());
  URV sr = aes_rv64_shiftrows_inv(r2, r1);
  URV result = aes_apply_inv_sbox_to_each_byte(sr);

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64dsm(const DecodedInst* di)
{
  if (not isRvzknd() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());
  uint64_t r2 = intRegs_.read(di->op2());
  uint64_t sr = aes_rv64_shiftrows_inv(r2, r1);
  uint64_t sb = aes_apply_inv_sbox_to_each_byte(sr);

  uint64_t low = aes_mixcolumn_inv(uint32_t(sb));
  uint64_t high = aes_mixcolumn_inv(uint32_t(sb >> 32));

  uint64_t result = (high << 32) | low;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64es(const DecodedInst* di)
{
  if (not isRvzkne() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());
  uint64_t r2 = intRegs_.read(di->op2());
  uint64_t sr = aes_rv64_shiftrows_fwd(r2, r1);
  uint64_t result = aes_apply_fwd_sbox_to_each_byte(sr);

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64esm(const DecodedInst* di)
{
  if (not isRvzkne() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());
  uint64_t r2 = intRegs_.read(di->op2());
  uint64_t sr = aes_rv64_shiftrows_fwd(r2, r1);
  uint64_t sb = aes_apply_fwd_sbox_to_each_byte(sr);

  uint64_t low = aes_mixcolumn_fwd(uint32_t(sb));
  uint64_t high = aes_mixcolumn_fwd(uint32_t(sb >> 32));

  uint64_t result = (high << 32) | low;

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64im(const DecodedInst* di)
{
  if (not isRvzknd() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());

  uint32_t w0 = aes_mixcolumn_inv(uint32_t(r1));
  uint32_t w1 = aes_mixcolumn_inv(uint32_t(r1 >> 32));
  uint64_t result = (uint64_t(w1) << 32) | uint64_t(w0);

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64ks1i(const DecodedInst* di)
{
  if (not isRvzkne() or not isRvzknd() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());
  unsigned rnum = di->op2();

  if ( rnum > 10)
    {
      illegalInst(di);
      return;
    }

  uint32_t tmp1 = r1 >> 32;
  uint32_t rc = aes_decode_rcon(rnum);

  uint32_t tmp2 = (rnum == 0xa) ? tmp1 : ror(tmp1, 8);
  uint32_t tmp3 = aes_subword_fwd(tmp2);
  uint32_t tmp4 = tmp3 ^ rc;

  uint64_t result = (uint64_t(tmp4) << 32) | uint64_t(tmp4);

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execAes64ks2(const DecodedInst* di)
{
  if (not isRvzkne() or not isRvzknd() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1 = intRegs_.read(di->op1());
  uint64_t r2 = intRegs_.read(di->op2());
  uint32_t w0 = uint32_t(r1 >> 32) ^ uint32_t(r2);
  uint32_t w1 = uint32_t(r1 >> 32) ^ uint32_t(r2) ^ uint32_t(r2 >> 32);
  uint64_t result = (uint64_t(w1) << 32) | uint64_t(w0);

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execSha256sig0(const DecodedInst* di)
{
  if (not isRvzknh())
    {
      illegalInst(di);
      return;
    }

  uint32_t inb      = intRegs_.read(di->op1());
  uint32_t result32 = ror(inb, 7) ^ ror(inb, 18) ^ (inb >> 3);

  URV result = static_cast<int32_t>(result32);  // sign extend.

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execSha256sig1(const DecodedInst* di)
{
  if (not isRvzknh())
    {
      illegalInst(di);
      return;
    }

  uint32_t inb      = intRegs_.read(di->op1());
  uint32_t result32 = ror(inb, 17) ^ ror(inb, 19) ^ (inb >> 10);

  URV result = static_cast<int32_t>(result32);  // sign extend.

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execSha256sum0(const DecodedInst* di)
{
  if (not isRvzknh())
    {
      illegalInst(di);
      return;
    }

  uint32_t inb      = intRegs_.read(di->op1());
  uint32_t result32 = ror(inb, 2) ^ ror(inb, 13) ^ ror(inb, 22);

  URV result = static_cast<int32_t>(result32);  // sign extend.

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execSha256sum1(const DecodedInst* di)
{
  if (not isRvzknh())
    {
      illegalInst(di);
      return;
    }

  uint32_t inb      = intRegs_.read(di->op1());
  uint32_t result32 = ror(inb, 6) ^ ror(inb, 11) ^ ror(inb, 25);

  URV result = static_cast<int32_t>(result32);  // sign extend.

  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execSha512sig0h(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }
  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 >>  1) ^ (r1 >>  7) ^ (r1 >>  8) ^
		   (r2 << 31)              ^ (r2 << 24) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sig0l(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 >>  1) ^ (r1 >>  7) ^ (r1 >>  8) ^
		   (r2 << 31) ^ (r2 << 25) ^ (r2 << 24) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sig1h(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 <<  3) ^ (r1 >>  6) ^ (r1 >> 19) ^
		   (r2 >> 29)              ^ (r2 << 13) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sig1l(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 <<  3) ^ (r1 >>  6) ^ (r1 >> 19) ^
		   (r2 >> 29) ^ (r2 << 26) ^ (r2 << 13) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sum0r(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 << 25) ^ (r1 << 30) ^ (r1 >> 28) ^
		   (r2 >>  7) ^ (r2 >>  2) ^ (r2 <<  4) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sum1r(const DecodedInst* di)
{
  if (not isRvzknh() or isRv64())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1 = intRegs_.read(di->op1());
  uint32_t r2 = intRegs_.read(di->op2());

  uint32_t res = ( (r1 << 23) ^ (r1 >> 14) ^ (r1 >> 18) ^
                   (r2 >>  9) ^ (r2 << 18) ^ (r2 << 14) );

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sig0(const DecodedInst* di)
{
  if (not isRvzknh() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1  = intRegs_.read(di->op1());
  uint64_t res = ror(r1, 1) ^ ror(r1, 8) ^ (r1 >> 7);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sig1(const DecodedInst* di)
{
  if (not isRvzknh() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1  = intRegs_.read(di->op1());
  uint64_t res = ror(r1, 19) ^ ror(r1, 61) ^ (r1 >> 6);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sum0(const DecodedInst* di)
{
  if (not isRvzknh() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1  = intRegs_.read(di->op1());
  uint64_t res = ror(r1, 28) ^ ror(r1, 34) ^ ror(r1, 39);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSha512sum1(const DecodedInst* di)
{
  if (not isRvzknh() or not isRv64())
    {
      illegalInst(di);
      return;
    }

  uint64_t r1  = intRegs_.read(di->op1());
  uint64_t res = ror(r1, 14) ^ ror(r1, 18) ^ ror(r1, 41);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSm3p0(const DecodedInst* di)
{
  if (not isRvzksh())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1     = intRegs_.read(di->op1());
  int32_t  sres32 = static_cast<int32_t>(r1 ^ rol(r1, 9) ^ rol(r1, 17));
  SRV      res    = sres32;  // sign extend.

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSm3p1(const DecodedInst* di)
{
  if (not isRvzksh())
    {
      illegalInst(di);
      return;
    }

  uint32_t r1     = intRegs_.read(di->op1());
  int32_t  sres32 = static_cast<int32_t>(r1 ^ rol(r1, 15) ^ rol(r1, 23));
  SRV      res    = sres32;  // sign extend.

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSm4ed(const DecodedInst* di)
{
  if (not isRvzksed())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3()* 8;
  uint8_t sb_in = uint32_t(intRegs_.read(di->op2())) >> shamt;
  uint32_t x = sm4_sbox(sb_in);
  uint32_t y = ( x ^ (x << 8)  ^ (x << 2) ^ (x << 18) ^ ((x & 0x3f) << 26) ^
		 ((x & 0xc0) << 10) );

  uint32_t z     = rol(y, shamt);
  uint32_t res32 = z ^ uint32_t(intRegs_.read(di->op1()));

  SRV res = int32_t(res32);  // sign extend
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSm4ks(const DecodedInst* di)
{
  if (not isRvzksed())
    {
      illegalInst(di);
      return;
    }
  if (not isRvzksed())
    {
      illegalInst(di);
      return;
    }

  unsigned shamt = di->op3()* 8;
  uint8_t sb_in = uint32_t(intRegs_.read(di->op2())) >> shamt;
  uint32_t x = sm4_sbox(sb_in);
  uint32_t y = ( x ^ ((x & 7) << 29)  ^ ((x & 0xfe) << 7) ^ ((x & 1) << 23) ^
		 ((x & 0xf8) << 13) );

  uint32_t z     = rol(y, shamt);
  uint32_t res32 = z ^ uint32_t(intRegs_.read(di->op1()));

  SRV res = int32_t(res32);  // sign extend
  intRegs_.write(di->op0(), res);
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
