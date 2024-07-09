#include "DecodedInst.hpp"
#include "Hart.hpp"
#include "crypto-util.hpp"

using namespace WdRiscv;


template <typename URV>
void
Hart<URV>::execClz(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());

  if (mxlen_ == 32)
    v1 = std::countl_zero(static_cast<uint32_t>(v1));
  else
    v1 = std::countl_zero(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execCtz(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());

  if (mxlen_ == 32)
    v1 = std::countr_zero(static_cast<uint32_t>(v1));
  else
    v1 = std::countr_zero(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execCpop(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());

  if (mxlen_ == 32)
    v1 = __builtin_popcount(v1);
  else
    v1 = __builtin_popcountl(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execClzw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  uint32_t v1 = intRegs_.read(di->op1());

  v1 = std::countl_zero(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execCtzw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  uint32_t v1 = intRegs_.read(di->op1());
  v1 = std::countr_zero(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execCpopw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  uint32_t v1 = intRegs_.read(di->op1());
  URV res = __builtin_popcount(v1);
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execAndn(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = v1 & ~v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execOrc_b(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  if (mxlen_ > sizeof(URV) * 8)
    {
      assert(0 and "mxlen is larger than xlen");
      return;
    }

  // GCC 12 optimizes the following down to couple of vector instructions
  // on Arm and x86_64.
  URV  urv   = intRegs_.read(di->op1());
  auto bytes = std::bit_cast<std::array<uint8_t, sizeof(URV)>>(urv);
  for (size_t i = 0; i < bytes.size(); i++)
    {
      bytes[i] = (bytes[i] == 0 ? 0 : UINT8_MAX);
    }
  urv = std::bit_cast<URV>(bytes);

  intRegs_.write(di->op0(), urv);
}


template <typename URV>
void
Hart<URV>::execOrn(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = v1 | ~v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execXnor(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = v1 ^ ~v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execMin(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  SRV v1 = intRegs_.read(di->op1());
  SRV v2 = intRegs_.read(di->op2());
  SRV res = v1 < v2? v1 : v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execMax(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  SRV v1 = intRegs_.read(di->op1());
  SRV v2 = intRegs_.read(di->op2());
  SRV res = v1 > v2? v1 : v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execMinu(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = v1 < v2? v1 : v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execMaxu(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = v1 > v2? v1 : v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execRol(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  URV rot = intRegs_.read(di->op2()) & mask;  // Rotate amount

  URV v1 = intRegs_.read(di->op1());
  URV res = (v1 << rot) | (v1 >> ((mxlen_ - rot) & mask));

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execRor(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  URV rot = intRegs_.read(di->op2()) & mask;  // Rotate amount

  URV v1 = intRegs_.read(di->op1());
  URV res = (v1 >> rot) | (v1 << ((mxlen_ - rot) & mask));

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execRori(const DecodedInst* di)
{
  if (not isRvzbb() and not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV rot = di->op2();
  if (not checkShiftImmediate(di, rot))
    return;

  URV v1 = intRegs_.read(di->op1());
  URV res = (v1 >> rot) | (v1 << ((mxlen_ - rot) & shiftMask()));

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execRolw(const DecodedInst* di)
{
  if (not isRv64() or (not isRvzbb() and not isRvzbkb()))
    {
      illegalInst(di);
      return;
    }

  unsigned len = 32;
  unsigned mask = len - 1;
  unsigned rot = intRegs_.read(di->op2()) & mask;  // Rotate amount

  uint32_t v1 = intRegs_.read(di->op1());
  uint32_t res32 = (v1 << rot) | (v1 >> ((len - rot) & mask));

  uint64_t res64 = int32_t(res32);  // Sign extend to 64-bits.

  intRegs_.write(di->op0(), res64);
}


template <typename URV>
void
Hart<URV>::execRorw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  unsigned len = 32;
  unsigned mask = len - 1;
  unsigned rot = intRegs_.read(di->op2()) & mask;  // Rotate amount

  uint32_t v1 = intRegs_.read(di->op1());
  uint32_t res32 = (v1 >> rot) | (v1 << ((len - rot) & mask));

  uint64_t res64 = int32_t(res32);  // Sign extend to 64-bits.

  intRegs_.write(di->op0(), res64);
}


template <typename URV>
void
Hart<URV>::execRoriw(const DecodedInst* di)
{
  if (not isRv64() or (not isRvzbb() and not isRvzbkb()))
    {
      illegalInst(di);
      return;
    }

  unsigned len = 32;
  unsigned mask = len - 1;
  unsigned rot = di->op2();

  uint32_t v1 = intRegs_.read(di->op1());
  uint32_t res32 = (v1 >> rot) | (v1 << ((len - rot) & mask));

  uint64_t res64 = int32_t(res32);  // Sign extend to 64-bits.

  intRegs_.write(di->op0(), res64);
}


template <typename URV>
void
Hart<URV>::execSext_b(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  int8_t byte = intRegs_.read(di->op1());
  SRV value = byte;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execSext_h(const DecodedInst* di)
{
  if (not isRvzbb())
    {
      illegalInst(di);
      return;
    }

  int16_t half = intRegs_.read(di->op1());
  SRV value = half;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execPack(const DecodedInst* di)
{
  // In RV32, zext.h is a Zbb pseudo-inst that maps to: pack rd, rs1, zero.
  bool zext_h = isRvzbb()  and  di->op2() == 0  and  not isRv64();
  bool legal = isRvzbkb() or zext_h;
  if (not legal)
    {
      illegalInst(di);
      return;
    }

  unsigned halfXlen = mxlen_ >> 1;
  URV lower = (intRegs_.read(di->op1()) << halfXlen) >> halfXlen;
  URV upper = intRegs_.read(di->op2()) << halfXlen;
  URV res = upper | lower;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSlli_uw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzba())
    {
      illegalInst(di);
      return;
    }

  uint32_t amount(di->op2());

  if (amount > 0x3f)
    {
      illegalInst(di);   // Bits 6 of immediate must be zero.
      return;
    }

  URV v1 = uint32_t(intRegs_.read(di->op1()));
  URV value = v1 << amount;

  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execPackh(const DecodedInst* di)
{
  if (not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV lower = intRegs_.read(di->op1()) & 0xff;
  URV upper = (intRegs_.read(di->op2()) & 0xff) << 8;
  URV value = lower | upper;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execPackw(const DecodedInst* di)
{
  if (not isRv64())
    {
      illegalInst(di);
      return;
    }

  // In RV64, zext.h is a Zbb pseudo-inst that maps to: packw rd, rs1, zero.
  bool zext_h = isRvzbb()  and  di->op2() == 0;
  bool legal = isRvzbkb() or zext_h;
  if (not legal)
    {
      illegalInst(di);
      return;
    }

  uint32_t v1 = intRegs_.read(di->op1());
  uint32_t v2 = intRegs_.read(di->op2());

  uint32_t lower = (v1 << 16) >> 16;
  uint32_t upper = v2 << 16;
  uint32_t value = lower | upper;

  int64_t res = int32_t(value);  // Sign extend.
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execBrev8(const DecodedInst* di)
{
  if (not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV result = brev8(v1);
  intRegs_.write(di->op0(), result);
}


template <typename URV>
void
Hart<URV>::execRev8_32(const DecodedInst* di)
{
  if ((not isRvzbb() and not isRvzbkb()) or isRv64())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());

  v1 = __builtin_bswap32(v1);

  intRegs_.write(di->op0(), v1);
}


template <typename URV>
void
Hart<URV>::execRev8_64(const DecodedInst* di)
{
  if ((not isRvzbb() and not isRvzbkb()) or not isRv64())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());

  v1 = __builtin_bswap64(v1);

  intRegs_.write(di->op0(), v1);
}


static
uint32_t
shuffleStage32(uint32_t src, uint32_t maskL, uint32_t maskR, unsigned n)
{
  uint32_t x = src & ~(maskL | maskR);
  x |= ((src << n) & maskL) | ((src >> n) & maskR);
  return x;
}


static
uint32_t
shuffle32(uint32_t x, unsigned shamt)
{
  if (shamt & 8)
    x = shuffleStage32(x, 0x00ff0000, 0x0000ff00, 8);
  if (shamt & 4)
    x = shuffleStage32(x, 0x0f000f00, 0x00f000f0, 4);
  if (shamt & 2)
    x = shuffleStage32(x, 0x30303030, 0x0c0c0c0c, 2);
  if (shamt & 1)
    x = shuffleStage32(x, 0x44444444, 0x22222222, 1);

  return x;
}


static
uint32_t
unshuffle32(uint32_t x, unsigned shamt)
{
  if (shamt & 1)
    x = shuffleStage32(x, 0x44444444, 0x22222222, 1);
  if (shamt & 2)
    x = shuffleStage32(x, 0x30303030, 0x0c0c0c0c, 2);
  if (shamt & 4)
    x = shuffleStage32(x, 0x0f000f00, 0x00f000f0, 4);
  if (shamt & 8)
    x = shuffleStage32(x, 0x00ff0000, 0x0000ff00, 8);

  return x;
}


#if 0

static
uint64_t
shuffleStage64(uint64_t src, uint64_t maskL, uint64_t maskR, unsigned n)
{
  uint64_t x = src & ~(maskL | maskR);
  x |= ((src << n) & maskL) | ((src >> n) & maskR);
  return x;
}


static
uint64_t
unshuffle64(uint64_t x, unsigned shamt)
{
  if (shamt & 1)
    x = shuffleStage64(x, 0x4444444444444444LL, 0x2222222222222222LL, 1);
  if (shamt & 2)
    x = shuffleStage64(x, 0x3030303030303030LL, 0x0c0c0c0c0c0c0c0cLL, 2);
  if (shamt & 4)
    x = shuffleStage64(x, 0x0f000f000f000f00LL, 0x00f000f000f000f0LL, 4);
  if (shamt & 8)
    x = shuffleStage64(x, 0x00ff000000ff0000LL, 0x0000ff000000ff00LL, 8);
  if (shamt & 16)
    x = shuffleStage64(x, 0x0000ffff00000000LL, 0x00000000ffff0000LL, 16);

  return x;
}

#endif


template <typename URV>
void
Hart<URV>::execBset(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  unsigned bitIx = intRegs_.read(di->op2()) & mask;

  URV value = intRegs_.read(di->op1()) | (URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBclr(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  unsigned bitIx = intRegs_.read(di->op2()) & mask;

  URV value = intRegs_.read(di->op1()) & ~(URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBinv(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  unsigned bitIx = intRegs_.read(di->op2()) & mask;

  URV value = intRegs_.read(di->op1()) ^ (URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBext(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV mask = shiftMask();
  unsigned bitIx = intRegs_.read(di->op2()) & mask;

  URV value = (intRegs_.read(di->op1()) >> bitIx) & 1;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBseti(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV bitIx = di->op2();
  if (not checkShiftImmediate(di, bitIx))
    return;

  URV value = intRegs_.read(di->op1()) | (URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBclri(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV bitIx = di->op2();
  if (not checkShiftImmediate(di, bitIx))
    return;

  URV value = intRegs_.read(di->op1()) & ~(URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBinvi(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV bitIx = di->op2();
  if (not checkShiftImmediate(di, bitIx))
    return;

  URV value = intRegs_.read(di->op1()) ^ (URV(1) << bitIx);
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execBexti(const DecodedInst* di)
{
  if (not isRvzbs())
    {
      illegalInst(di);
      return;
    }

  URV bitIx = di->op2();
  if (not checkShiftImmediate(di, bitIx))
    return;

  URV value = (intRegs_.read(di->op1()) >> bitIx) & 1;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
void
Hart<URV>::execClmul(const DecodedInst* di)
{
  if (not isRvzbc())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV x = 0;
  for (unsigned i = 0; i < mxlen_; ++i)
    if ((v2 >> i) & 1)
      x ^= v1 << i;

  intRegs_.write(di->op0(), x);
}


template <typename URV>
void
Hart<URV>::execClmulh(const DecodedInst* di)
{
  if (not isRvzbc())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV x = 0;
  for (unsigned i = 1; i < mxlen_; ++i)
    if ((v2 >> i) & 1)
      x ^= v1 >> (mxlen_ - i);

  intRegs_.write(di->op0(), x);
}


template <typename URV>
void
Hart<URV>::execUnzip(const DecodedInst* di)
{
  if (isRv64() or not isRvzbkb())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV val = unshuffle32(v1, 0xf);

  intRegs_.write(di->op0(), val);
}


template <typename URV>
void
Hart<URV>::execZip(const DecodedInst* di)
{
  if (not isRvzbkb() or isRv64())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV val = shuffle32(v1, 0xf);

  intRegs_.write(di->op0(), val);
}


static
uint32_t
xperm32(uint32_t v1, uint32_t v2, unsigned log2Width)
{
  uint32_t res = 0;
  uint32_t width = 1u << log2Width;
  uint32_t mask = (1u << width) - 1;
  for (unsigned i = 0; i < 32; i += width)
    {
      uint32_t pos = ((v2 >> i) & mask) << log2Width;
      if (pos < 32)
        res |= ((v1 >> pos) & mask) << i;
    }
  return res;
}

  
static
uint64_t
xperm64(uint64_t v1, uint64_t v2, unsigned log2Width)
{
  uint64_t res = 0;
  uint32_t width = 1u << log2Width;
  uint64_t mask = (uint64_t(1) << width) - 1;
  for (unsigned i = 0; i < 64; i += width)
    {
      uint64_t pos = ((v2 >> i) & mask) << log2Width;
      if (pos < 64)
        res |= ((v1 >> pos) & mask) << i;
    }
  return res;
}


template <typename URV>
void
Hart<URV>::execXperm_n(const DecodedInst* di)
{
  if (not isRvzbkx())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = 0;

  if (mxlen_ == 32)
    res = xperm32(v1, v2, 2);
  else
    res = xperm64(v1, v2, 2);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execXperm_b(const DecodedInst* di)
{
  if (not isRvzbkx())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());
  URV res = 0;

  if (mxlen_ == 32)
    res = xperm32(v1, v2, 3);
  else
    res = xperm64(v1, v2, 3);

  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execClmulr(const DecodedInst* di)
{
  if (not isRvzbc())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV x = 0;
  for (unsigned i = 0; i < mxlen_; ++i)
    if ((v2 >> i) & 1)
      x ^= v1 >> (mxlen_ - i - 1);

  intRegs_.write(di->op0(), x);
}


template <typename URV>
void
Hart<URV>::execSh1add(const DecodedInst* di)
{
  if (not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 1) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSh2add(const DecodedInst* di)
{
  if (not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 2) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSh3add(const DecodedInst* di)
{
  if (not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = intRegs_.read(di->op1());
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 3) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSh1add_uw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = uint32_t(intRegs_.read(di->op1()));
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 1) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSh2add_uw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = uint32_t(intRegs_.read(di->op1()));
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 2) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execSh3add_uw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = uint32_t(intRegs_.read(di->op1()));
  URV v2 = intRegs_.read(di->op2());

  URV res = (v1 << 3) + v2;
  intRegs_.write(di->op0(), res);
}


template <typename URV>
void
Hart<URV>::execAdd_uw(const DecodedInst* di)
{
  if (not isRv64() or not isRvzba())
    {
      illegalInst(di);
      return;
    }

  URV v1 = uint32_t(intRegs_.read(di->op1()));
  URV v2 = intRegs_.read(di->op2());

  URV value = v1 + v2;
  intRegs_.write(di->op0(), value);
}


template <typename URV>
static
URV
crc32(URV x, unsigned nbits)
{
  for (unsigned i = 0; i < nbits; ++i)
    x = (x >> 1) ^ (0xedb88320 & ~((x & 1) - 1));
  return x;
}


template <typename URV>
static
URV
crc32c(URV x, unsigned nbits)
{
  for (unsigned i = 0; i < nbits; ++i)
    x = (x >> 1) ^ (0x82F63B78  & ~((x & 1) - 1));
  return x;
}


template class WdRiscv::Hart<uint32_t>;
template class WdRiscv::Hart<uint64_t>;
