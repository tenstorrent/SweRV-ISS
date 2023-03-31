#include <iostream>
#include <cstdint>
#include "Hart.hpp"

using namespace WdRiscv;


static inline
uint8_t
xt2(uint8_t x)
{
  uint8_t res = (x << 1) ^ ((x >> 7) ? 0x1b : 0);
  return res;
}
    

static inline
uint8_t
xt3(uint8_t x)
{
  return x ^ xt2(x);
}


static inline
uint8_t
gfmul(uint8_t x, uint8_t y)
{
  uint8_t res = (y & 1) ? x : 0;
  res = res ^ ((y & 2) ? xt2(x) : 0);
  res = res ^ ((y & 4) ? xt2(xt2(x)) : 0);
  res = res ^ ((y & 8) ? xt2(xt2(xt2(x))) : 0);
  return res;
}


template <typename uint_t>
static constexpr
uint_t
rol(uint_t in, unsigned amount)
{
  return ((in << amount) | (in >> (std::numeric_limits<uint_t>::digits - amount)));
}


template <typename uint_t>
static constexpr
uint_t
ror(uint_t in, unsigned amount)
{
  return ((in >> amount) | (in << (std::numeric_limits<uint_t>::digits - amount)));
}


static inline
uint32_t
aes_mixcolumn_byte_fwd(uint8_t so)
{
  return ( (uint32_t(gfmul(so, 0x3)) << 24) |
	   (uint32_t(so) << 16)             |
	   (uint32_t(so) << 8)              |
	   (uint32_t(gfmul(so, 0x2))) );
}

static inline
uint32_t
aes_mixcolumn_byte_inv(uint8_t so)
{
  return ( (uint32_t(gfmul(so, 0xb)) << 24) |
	   (uint32_t(gfmul(so, 0xd)) << 16) |
	   (uint32_t(gfmul(so, 0x9)) << 8)  |
	   (uint32_t(gfmul(so, 0xe))) );
}

/* 32-bit to 32-bit AES forward MixColumn */
static
uint32_t
aes_mixcolumn_fwd(uint32_t x)
{
  uint8_t s0 = x;
  uint8_t s1 = x >> 8;
  uint8_t s2 = x >> 16;
  uint8_t s3 = x >> 24;
  uint8_t b0 = xt2(s0) ^ xt3(s1) ^ s2 ^ s3;
  uint8_t b1 = s0 ^ xt2(s1) ^ xt3(s2) ^ s3;
  uint8_t b2 = s0 ^ s1 ^ xt2(s2) ^ xt3(s3);
  uint8_t b3 = xt3(s0) ^ s1 ^ s2 ^ xt2(s3);
  return (uint32_t(b3) << 24) | (uint32_t(b2) << 16) | (uint32_t(b1) << 8) | b0;
}


static
uint32_t
aes_mixcolumn_inv(uint32_t x)
{
  uint8_t s0 = x;
  uint8_t s1 = x >> 8;
  uint8_t s2 = x >> 16;
  uint8_t s3 = x >> 24;
  uint8_t b0 = gfmul(s0, 0xE) ^ gfmul(s1, 0xB) ^ gfmul(s2, 0xD) ^ gfmul(s3, 0x9);
  uint8_t b1 = gfmul(s0, 0x9) ^ gfmul(s1, 0xE) ^ gfmul(s2, 0xB) ^ gfmul(s3, 0xd);
  uint8_t b2 = gfmul(s0, 0xD) ^ gfmul(s1, 0x9) ^ gfmul(s2, 0xE) ^ gfmul(s3, 0xb);
  uint8_t b3 = gfmul(s0, 0xB) ^ gfmul(s1, 0xD) ^ gfmul(s2, 0x9) ^ gfmul(s3, 0xe);
  return (uint32_t(b3) << 24) | (uint32_t(b2) << 16) | (uint32_t(b1) << 8) | b0;
}


static
uint32_t
aes_decode_rcon(uint8_t r)
{
  static uint32_t table[16] =
    {
      0x00000001,
      0x00000002,
      0x00000004,
      0x00000008,
      0x00000010,
      0x00000020,
      0x00000040,
      0x00000080,
      0x0000001b,
      0x00000036,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000,
      0x00000000
    };

  return table[r & 0xf];
}


static
uint8_t
sm4_sbox(uint8_t x)
{
  uint8_t sm4_sbox_table[256] =
    {
      0xD6, 0x90, 0xE9, 0xFE, 0xCC, 0xE1, 0x3D, 0xB7, 0x16, 0xB6, 0x14, 0xC2, 0x28,
      0xFB, 0x2C, 0x05, 0x2B, 0x67, 0x9A, 0x76, 0x2A, 0xBE, 0x04, 0xC3, 0xAA, 0x44,
      0x13, 0x26, 0x49, 0x86, 0x06, 0x99, 0x9C, 0x42, 0x50, 0xF4, 0x91, 0xEF, 0x98,
      0x7A, 0x33, 0x54, 0x0B, 0x43, 0xED, 0xCF, 0xAC, 0x62, 0xE4, 0xB3, 0x1C, 0xA9,
      0xC9, 0x08, 0xE8, 0x95, 0x80, 0xDF, 0x94, 0xFA, 0x75, 0x8F, 0x3F, 0xA6, 0x47,
      0x07, 0xA7, 0xFC, 0xF3, 0x73, 0x17, 0xBA, 0x83, 0x59, 0x3C, 0x19, 0xE6, 0x85,
      0x4F, 0xA8, 0x68, 0x6B, 0x81, 0xB2, 0x71, 0x64, 0xDA, 0x8B, 0xF8, 0xEB, 0x0F,
      0x4B, 0x70, 0x56, 0x9D, 0x35, 0x1E, 0x24, 0x0E, 0x5E, 0x63, 0x58, 0xD1, 0xA2,
      0x25, 0x22, 0x7C, 0x3B, 0x01, 0x21, 0x78, 0x87, 0xD4, 0x00, 0x46, 0x57, 0x9F,
      0xD3, 0x27, 0x52, 0x4C, 0x36, 0x02, 0xE7, 0xA0, 0xC4, 0xC8, 0x9E, 0xEA, 0xBF,
      0x8A, 0xD2, 0x40, 0xC7, 0x38, 0xB5, 0xA3, 0xF7, 0xF2, 0xCE, 0xF9, 0x61, 0x15,
      0xA1, 0xE0, 0xAE, 0x5D, 0xA4, 0x9B, 0x34, 0x1A, 0x55, 0xAD, 0x93, 0x32, 0x30,
      0xF5, 0x8C, 0xB1, 0xE3, 0x1D, 0xF6, 0xE2, 0x2E, 0x82, 0x66, 0xCA, 0x60, 0xC0,
      0x29, 0x23, 0xAB, 0x0D, 0x53, 0x4E, 0x6F, 0xD5, 0xDB, 0x37, 0x45, 0xDE, 0xFD,
      0x8E, 0x2F, 0x03, 0xFF, 0x6A, 0x72, 0x6D, 0x6C, 0x5B, 0x51, 0x8D, 0x1B, 0xAF,
      0x92, 0xBB, 0xDD, 0xBC, 0x7F, 0x11, 0xD9, 0x5C, 0x41, 0x1F, 0x10, 0x5A, 0xD8,
      0x0A, 0xC1, 0x31, 0x88, 0xA5, 0xCD, 0x7B, 0xBD, 0x2D, 0x74, 0xD0, 0x12, 0xB8,
      0xE5, 0xB4, 0xB0, 0x89, 0x69, 0x97, 0x4A, 0x0C, 0x96, 0x77, 0x7E, 0x65, 0xB9,
      0xF1, 0x09, 0xC5, 0x6E, 0xC6, 0x84, 0x18, 0xF0, 0x7D, 0xEC, 0x3A, 0xDC, 0x4D,
      0x20, 0x79, 0xEE, 0x5F, 0x3E, 0xD7, 0xCB, 0x39, 0x48
    };

  return sm4_sbox_table[x];
}


static
uint8_t
aes_sbox_fwd(uint8_t x)
{
  static uint8_t aes_sbox_fwd_table[256] =
    {
      0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe,
      0xd7, 0xab, 0x76, 0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4,
      0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, 0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7,
      0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3,
      0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, 0x09,
      0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3,
      0x2f, 0x84, 0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe,
      0x39, 0x4a, 0x4c, 0x58, 0xcf, 0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85,
      0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, 0x51, 0xa3, 0x40, 0x8f, 0x92,
      0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, 0xcd, 0x0c,
      0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19,
      0x73, 0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14,
      0xde, 0x5e, 0x0b, 0xdb, 0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2,
      0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, 0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5,
      0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, 0xba, 0x78, 0x25,
      0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
      0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86,
      0xc1, 0x1d, 0x9e, 0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e,
      0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, 0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42,
      0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
    };

  return aes_sbox_fwd_table[x];
}


static
uint8_t
aes_sbox_inv(uint8_t x)
{
  static uint8_t aes_sbox_inv_table[256] =
    {
      0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81,
      0xf3, 0xd7, 0xfb, 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e,
      0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb, 0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23,
      0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e, 0x08, 0x2e, 0xa1, 0x66,
      0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25, 0x72,
      0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65,
      0xb6, 0x92, 0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46,
      0x57, 0xa7, 0x8d, 0x9d, 0x84, 0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a,
      0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06, 0xd0, 0x2c, 0x1e, 0x8f, 0xca,
      0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b, 0x3a, 0x91,
      0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6,
      0x73, 0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8,
      0x1c, 0x75, 0xdf, 0x6e, 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f,
      0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b, 0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2,
      0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4, 0x1f, 0xdd, 0xa8,
      0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
      0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93,
      0xc9, 0x9c, 0xef, 0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb,
      0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61, 0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6,
      0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
    };

  return aes_sbox_inv_table[x];
}


static
uint32_t
aes_subword_fwd(uint32_t x)
{
  uint32_t y3 = uint32_t(aes_sbox_fwd(uint8_t(x >> 24))) << 24;
  uint32_t y2 = uint32_t(aes_sbox_fwd(uint8_t(x >> 16))) << 16;
  uint32_t y1 = uint32_t(aes_sbox_fwd(uint8_t(x >> 8)))  << 8;
  uint32_t y0 = aes_sbox_fwd(uint8_t(x));
  return y3 | y2 | y1 | y0;
}


#if 0
static
uint32_t
aes_subword_inv(uint32_t x)
{
  uint32_t y3 = uint32_t(aes_sbox_inv(uint8_t(x >> 24))) << 24;
  uint32_t y2 = uint32_t(aes_sbox_inv(uint8_t(x >> 16))) << 16;
  uint32_t y1 = uint32_t(aes_sbox_inv(uint8_t(x >> 8)))  << 8;
  uint32_t y0 = aes_sbox_inv(uint8_t(x));
  return y3 | y2 | y1 | y0;
}


static inline
uint32_t
aes_get_column(__uint128_t state, uint8_t c)
{
  uint32_t res = state >> (32*c & 0x7f);
  return res;
}
#endif


static
uint64_t
aes_apply_fwd_sbox_to_each_byte(uint64_t x)
{
  uint64_t y7 = uint64_t(aes_sbox_fwd(uint8_t(x >> 56))) << 56;
  uint64_t y6 = uint64_t(aes_sbox_fwd(uint8_t(x >> 48))) << 48;
  uint64_t y5 = uint64_t(aes_sbox_fwd(uint8_t(x >> 40))) << 40;
  uint64_t y4 = uint64_t(aes_sbox_fwd(uint8_t(x >> 32))) << 32;
  uint64_t y3 = uint64_t(aes_sbox_fwd(uint8_t(x >> 24))) << 24;
  uint64_t y2 = uint64_t(aes_sbox_fwd(uint8_t(x >> 16))) << 16;
  uint64_t y1 = uint64_t(aes_sbox_fwd(uint8_t(x >>  8))) <<  8;
  uint64_t y0 = uint64_t(aes_sbox_fwd(uint8_t(x)));
  return y7 | y6 | y5 | y4 | y3 | y2 | y1 | y0;
}


static
uint64_t
aes_apply_inv_sbox_to_each_byte(uint64_t x)
{
  uint64_t y7 = uint64_t(aes_sbox_inv(uint8_t(x >> 56))) << 56;
  uint64_t y6 = uint64_t(aes_sbox_inv(uint8_t(x >> 48))) << 48;
  uint64_t y5 = uint64_t(aes_sbox_inv(uint8_t(x >> 40))) << 40;
  uint64_t y4 = uint64_t(aes_sbox_inv(uint8_t(x >> 32))) << 32;
  uint64_t y3 = uint64_t(aes_sbox_inv(uint8_t(x >> 24))) << 24;
  uint64_t y2 = uint64_t(aes_sbox_inv(uint8_t(x >> 16))) << 16;
  uint64_t y1 = uint64_t(aes_sbox_inv(uint8_t(x >>  8))) <<  8;
  uint64_t y0 = uint64_t(aes_sbox_inv(uint8_t(x)));
  return y7 | y6 | y5 | y4 | y3 | y2 | y1 | y0;
}


static inline
uint8_t
getbyte(uint64_t x, unsigned i)
{
  return x >> (i*8);
}


static
uint64_t
aes_rv64_shiftrows_fwd(uint64_t rs2, uint64_t rs1)
{
  uint64_t y7 = uint64_t(getbyte(rs1, 3)) << 56;
  uint64_t y6 = uint64_t(getbyte(rs2, 6)) << 48;
  uint64_t y5 = uint64_t(getbyte(rs2, 1)) << 40;
  uint64_t y4 = uint64_t(getbyte(rs1, 4)) << 32;
  uint64_t y3 = uint64_t(getbyte(rs2, 7)) << 24;
  uint64_t y2 = uint64_t(getbyte(rs2, 2)) << 16;
  uint64_t y1 = uint64_t(getbyte(rs1, 5)) <<  8;
  uint64_t y0 = uint64_t(getbyte(rs1, 0));
  return y7 | y6 | y5 | y4 | y3 | y2 | y1 | y0;
}


static
uint64_t
aes_rv64_shiftrows_inv(uint64_t rs2, uint64_t rs1)
{
  uint64_t y7 = uint64_t(getbyte(rs2, 3)) << 56;
  uint64_t y6 = uint64_t(getbyte(rs2, 6)) << 48;
  uint64_t y5 = uint64_t(getbyte(rs1, 1)) << 40;
  uint64_t y4 = uint64_t(getbyte(rs1, 4)) << 32;
  uint64_t y3 = uint64_t(getbyte(rs1, 7)) << 24;
  uint64_t y2 = uint64_t(getbyte(rs2, 2)) << 16;
  uint64_t y1 = uint64_t(getbyte(rs2, 5)) <<  8;
  uint64_t y0 = uint64_t(getbyte(rs1, 0));
  return y7 | y6 | y5 | y4 | y3 | y2 | y1 | y0;
}


#if 0
static
__uint128_t
aes_shift_rows_fwd(__uint128_t x)
{
  uint32_t ic3 = aes_get_column(x, 3);
  uint32_t ic2 = aes_get_column(x, 2);
  uint32_t ic1 = aes_get_column(x, 1);
  uint32_t ic0 = aes_get_column(x, 0);

  uint32_t oc0 = ( ((ic0 >> 24) << 24) | (ic1 & uint32_t(0xff0000)) |
		   (ic2 & uint32_t(0xff00)) | (ic3 & uint32_t(0xff)) );

  uint32_t oc1 = ( ((ic1 >> 14) << 24) | (ic2 & uint32_t(0xff0000)) |
		   (ic3 & uint32_t(0xff00)) | (ic0 & uint32_t(0xff)) );

  uint32_t oc2 = ( ((ic2 >> 24) << 24) | (ic3 & uint32_t(0xff0000)) |
		   (ic0 & uint32_t(0xff00)) | (ic1 & uint32_t(0xff)) );

  uint32_t oc3 = ( ((ic3 >> 24) << 24) | (ic0 & uint32_t(0xff0000)) |
		   (ic1 & uint32_t(0xff00)) | (ic2 & uint32_t(0xff)) );

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}


static
__uint128_t
aes_shift_rows_inv(__uint128_t x)
{
  uint32_t ic3 = aes_get_column(x, 3);
  uint32_t ic2 = aes_get_column(x, 2);
  uint32_t ic1 = aes_get_column(x, 1);
  uint32_t ic0 = aes_get_column(x, 0);

  uint32_t oc0 = ( ((ic0 >> 24) << 24) | (ic3 & uint32_t(0xff0000)) |
		   (ic2 & uint32_t(0xff00)) | (ic1 & uint32_t(0xff)) );

  uint32_t oc1 = ( ((ic1 >> 14) << 24) | (ic0 & uint32_t(0xff0000)) |
		   (ic3 & uint32_t(0xff00)) | (ic2 & uint32_t(0xff)) );

  uint32_t oc2 = ( ((ic2 >> 24) << 24) | (ic1 & uint32_t(0xff0000)) |
		   (ic0 & uint32_t(0xff00)) | (ic3 & uint32_t(0xff)) );

  uint32_t oc3 = ( ((ic3 >> 24) << 24) | (ic2 & uint32_t(0xff0000)) |
		   (ic1 & uint32_t(0xff00)) | (ic0 & uint32_t(0xff)) );

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}


static
__uint128_t
aes_subbytes_fwd(__uint128_t x)
{
  uint32_t oc0 = aes_subword_fwd(aes_get_column(x, 0));
  uint32_t oc1 = aes_subword_fwd(aes_get_column(x, 1));
  uint32_t oc2 = aes_subword_fwd(aes_get_column(x, 2));
  uint32_t oc3 = aes_subword_fwd(aes_get_column(x, 3));

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}


static
__uint128_t
aes_subbytes_inv(__uint128_t x)
{
  uint32_t oc0 = aes_subword_inv(aes_get_column(x, 0));
  uint32_t oc1 = aes_subword_inv(aes_get_column(x, 1));
  uint32_t oc2 = aes_subword_inv(aes_get_column(x, 2));
  uint32_t oc3 = aes_subword_inv(aes_get_column(x, 3));

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}


static
__uint128_t
aes_mixcolumns_fwd(__uint128_t x)
{
  uint32_t oc0 = aes_mixcolumn_fwd(aes_get_column(x, 0));
  uint32_t oc1 = aes_mixcolumn_fwd(aes_get_column(x, 1));
  uint32_t oc2 = aes_mixcolumn_fwd(aes_get_column(x, 2));
  uint32_t oc3 = aes_mixcolumn_fwd(aes_get_column(x, 3));

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}


static
__uint128_t
aes_mixcolumns_inv(__uint128_t x)
{
  uint32_t oc0 = aes_mixcolumn_inv(aes_get_column(x, 0));
  uint32_t oc1 = aes_mixcolumn_inv(aes_get_column(x, 1));
  uint32_t oc2 = aes_mixcolumn_inv(aes_get_column(x, 2));
  uint32_t oc3 = aes_mixcolumn_inv(aes_get_column(x, 3));

  __uint128_t res = ( (__uint128_t(oc3) << 96) | (__uint128_t(oc2) << 64) |
		      (__uint128_t(oc1) << 32) | __uint128_t(oc0) );
  return res;
}
#endif

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

  uint32_t res = ( (r1 << 23) ^ (r1 << 14) ^ (r1 >> 18) ^
		   (r2 >>  9) ^ (r2 >> 18) ^ (r2 << 14) );

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
  int32_t  sres32 = r1 ^ rol(r1, 9) ^ rol(r1, 17);
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
  int32_t  sres32 = r1 ^ rol(r1, 15) ^ rol(r1, 23);
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
