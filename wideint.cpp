#undef __SIZEOF_INT128__

#include <string_view>
#include "wideint.hpp"

using namespace WdRiscv;


# if 0

#include <cassert>
#include <iostream>
#include <random>


template <typename T>
requires std::derived_from<T, WideIntBase>
std::string
toHexString(T x)
{
  typedef typename std::make_unsigned<T>::type U;
  U u = x;
  std::string res;
  for ( ; u; u >>= 4)
    {
      unsigned hd = unsigned(u & 0xf);
      res.push_back("0123456789abcdef"[hd]);
    }
  std::reverse(res.begin(), res.end());
  return res;
}


template <typename T>
T
fromHexString(const std::string& str)
{
  if (str.empty())
    return 0;

  std::string_view sv(str);
  if (str.size() > 2 and (str.starts_with("0x") or (str.starts_with("0X"))))
    sv = std::string_view(str.begin() + 2, str.end());

  T res{};
  unsigned i = 0;
  for (auto it = sv.rbegin(); it != sv.rend(); ++it, ++i)
    {
      char c = *it;
      unsigned v = 0;
      if (c >= '0' and c <= '9')
	v = c - '0';
      else if (c >= 'a' and c <= 'f')
	v = c - 'a' + 10;
      else if (c >= 'A' and c <= 'F')
	v = c - 'A' + 10;
      else
	throw std::runtime_error("Unexpected character in hexadecimal string");
      res |= Uint128(v) << (i*4);
    }
  return res;
}


template <typename T>
requires std::derived_from<T, WideIntBase>
std::string
toDecimalString(T x)
{
  typedef typename std::make_unsigned<T>::type U;
  U u = x;
  std::string res;
  while (u)
    {
      unsigned d = u % 10;
      res.push_back("0123456789"[d]);
      u /= 10;
    }
  std::reverse(res.begin(), res.end());
  return res;
}


template <typename T>
T
fromDecimalString(const std::string& str)
{
  if (str.empty())
    return 0;

  std::string_view sv(str);

  T res{};
  T weight{1};
  for (auto it = sv.rbegin(); it != sv.rend(); ++it, weight *= 10)
    {
      char c = *it;
      unsigned v = 0;
      if (c >= '0' and c <= '9')
	v = c - '0';
      else
	throw std::runtime_error("Unexpected character in hexadecimal string");
      res += v*weight;
    }
  return res;
}


int
main(int argc, char* argv[])
{
  uint64_t ui64 = 0;
  Int128 i128 = 0xffffffffffffffffLL;
  i128 <<= 64;
  i128 = 1 + i128;
  i128 = 1 * i128;
  i128 = 1 / i128;
  i128 = 1 % i128;
  i128 = 0x404040404040404LL;
  i128 <<= 64;
  i128 |= 0x404040404040404LL;
  i128 = i128 >> 2;
  i128 = i128 | 4;
  i128 = 4 | i128;
  i128 = i128 & 4;
  i128 = 4 & i128;
  i128 = i128 ^ 4;
  i128 = 4 ^ i128;
  i128 = +i128;

  std::cerr << toHexString(i128) << '\n';

  i128 = fromHexString<Int128>("0x1234567890abcdef1234567890abcdef");
  std::cerr << toHexString(i128) << '\n';

  i128 = fromDecimalString<Int128>("1024");
  std::cerr << toHexString(i128) << '\n';
  std::cerr << toDecimalString(i128) << '\n';

  Int256 i256 = 0;
  i256 = ~i256;
  int64_t i64 = int64_t(i256);

  Uint128 u128 = 0; u128 = ~u128;
  unsigned bit = unsigned((u128 >> 127) & 1);
  u128 = u128 >> 127;
  i256 = u128;
  i256 = -1;
  Int512 i512{i256};
  bool d = i512 != 0;
  d = u128 != 0;
  
  uint64_t a = ~uint64_t(0);
  __uint128_t c = ~ __uint128_t(0);
  c *= a;

  Uint128 cc = a;
  cc = (cc << 64) | a;
  cc *= a;

  assert(cc.high() == uint64_t(c >> 64));
  assert(cc.low() == (c << 64) >> 64);

  c = ~ __uint128_t(0);
  c *= c;

  cc = Uint128(a, a);
  cc *= cc;

  if ((cc.high() != c >> 64) or (cc.low() != (c << 64) >> 64))
    std::cerr << "Failed\n";

  c = c + c;
  cc = cc + cc;
  if ((cc.high() != c >> 64) or (cc.low() != (c << 64) >> 64))
    std::cerr << "Failed\n";

  Uint256 u256 = cc;
  u256 = u256 + u256;
  u256 = u256 + 1;

  cc = cc - cc;
  if ((cc.high() != 0) or (cc.low() != 0))
    std::cerr << "Failed\n";

  std::mt19937 gen;
  std::uniform_int_distribution<uint64_t> distrib(0, ~uint64_t(0));

  for (unsigned i = 0; i < 100000000; ++i)
    {
      if ((i % 1000000) == 0)
        std::cerr << std::dec << i << '\n';

      uint64_t low1 = distrib(gen), high1 = distrib(gen);
      __uint128_t x1 = high1;
      x1 = (x1 << 64) | low1;
      Uint128 y1(high1, low1);

      uint64_t low2 = distrib(gen), high2 = distrib(gen);
      __uint128_t x2 = high2;
      x2 = (x2 << 64) | low2;
      Uint128 y2(high2, low2);

      __uint128_t z1 = x1 * x2;
      Uint128 z2 = y1 * y2;

      if ((z2.high() != z1 >> 64) or (z2.low() != (z1 << 64) >> 64))
        {
          std::cerr << std::dec << "Failed mulu " << i << std::hex << " on 0x"
                    << high1 << " 0x" << low1
                    << " * 0x" << high2 << " 0x " << low2 << '\n';
          return 1;
        }

      __int128_t sx1 = x1, sx2 = x2;
      __int128_t sz1 = sx1 * sx2;

      Int128 sy1 = y1, sy2 = y2;
      Int128 sz2 = sy1 * sy2;

      if ((sz2.high() != sz1 >> 64 or z2.low() != (z1 << 64) >> 64))
        {
          std::cerr << std::dec << "Failed mul " << i << std::hex << " on 0x"
                    << high1 << " 0x" << low1
                    << " * 0x" << high2 << " 0x " << low2 << '\n';
          return 1;
        }

      if (x2 == 0)
        continue;

      z1 = x1 / x2;
      z2 = y1 / y2;

      if ((z2.high() != z1 >> 64) or (z2.low() != (z1 << 64) >> 64))
        {
          std::cerr << std::dec << "Failed divu" << i << std::hex << " on 0x"
                    << high1 << " 0x" << low1
                    << " * 0x" << high2 << " 0x " << low2 << '\n';
          return 1;
        }

      sz1 = sx1 / sx2;
      sz2 = sy1 / sy2;

      if ((sz2.high() != sz1 >> 64) or (sz2.low() != (sz1 << 64) >> 64))
        {
          std::cerr << std::dec << "Failed div" << i << std::hex << " on 0x"
                    << high1 << " 0x" << low1
                    << " * 0x" << high2 << " 0x " << low2 << '\n';
          return 1;
        }

    }

  Uint128 a128 = Uint128{0xfcfcfcfc, 0xfcfcfcfc};
  Uint128 b128 = Uint128{0x03030303, 0x03030303};
  Uint512 a512(a128, a128);
  Uint512 b512(b128, b128);

  Uint512 c512 = b512*Uint512(4);

  return 0;
}

#endif
