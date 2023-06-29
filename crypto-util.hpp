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

#pragma once

#include <array>
#include <bit>
#include <cstdint>
#include <limits>
#include <tuple>
#include "wideint.hpp"

constexpr
uint8_t
xt2(uint8_t x)
{
  uint8_t res = (x << 1) ^ ((x >> 7) ? 0x1b : 0);
  return res;
}


constexpr
uint8_t
xt3(uint8_t x)
{
  return x ^ xt2(x);
}


constexpr
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
constexpr
uint_t
rol(uint_t in, unsigned amount)
{
  return ((in << amount) | (in >> (std::numeric_limits<uint_t>::digits - amount)));
}


template <typename uint_t>
constexpr
uint_t
ror(uint_t in, unsigned amount)
{
  return ((in >> amount) | (in << (std::numeric_limits<uint_t>::digits - amount)));
}


constexpr
uint32_t
aes_mixcolumn_byte_fwd(uint8_t so)
{
  return WdRiscv::fromQuarters(gfmul(so, 0x2),
                               so,
                               so,
                               gfmul(so, 0x3));
}

constexpr
uint32_t
aes_mixcolumn_byte_inv(uint8_t so)
{
  return WdRiscv::fromQuarters(gfmul(so, 0xe),
                               gfmul(so, 0x9),
                               gfmul(so, 0xd),
                               gfmul(so, 0xb));
}

/* 32-bit to 32-bit AES forward MixColumn */
constexpr
uint32_t
aes_mixcolumn_fwd(uint32_t x)
{
  auto [s0, s1, s2, s3] = WdRiscv::toQuarters(x);
  uint8_t b0            = xt2(s0) ^ xt3(s1) ^ s2 ^ s3;
  uint8_t b1            = s0 ^ xt2(s1) ^ xt3(s2) ^ s3;
  uint8_t b2            = s0 ^ s1 ^ xt2(s2) ^ xt3(s3);
  uint8_t b3            = xt3(s0) ^ s1 ^ s2 ^ xt2(s3);
  return WdRiscv::fromQuarters(b0, b1, b2, b3);
}


constexpr
uint32_t
aes_mixcolumn_inv(uint32_t x)
{
  auto [s0, s1, s2, s3] = WdRiscv::toQuarters(x);
  uint8_t b0            = gfmul(s0, 0xE) ^ gfmul(s1, 0xB) ^ gfmul(s2, 0xD) ^ gfmul(s3, 0x9);
  uint8_t b1            = gfmul(s0, 0x9) ^ gfmul(s1, 0xE) ^ gfmul(s2, 0xB) ^ gfmul(s3, 0xd);
  uint8_t b2            = gfmul(s0, 0xD) ^ gfmul(s1, 0x9) ^ gfmul(s2, 0xE) ^ gfmul(s3, 0xb);
  uint8_t b3            = gfmul(s0, 0xB) ^ gfmul(s1, 0xD) ^ gfmul(s2, 0x9) ^ gfmul(s3, 0xe);
  return WdRiscv::fromQuarters(b0, b1, b2, b3);
}


constexpr
uint32_t
aes_decode_rcon(uint8_t r)
{
  constexpr auto table = std::to_array<uint32_t>(
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
    });
  static_assert(table.size() == 16);

  return table[r & 0xf];
}


constexpr
uint8_t
sm4_sbox(uint8_t x)
{
  constexpr auto sm4_sbox_table = std::to_array<uint8_t>(
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
    });
  static_assert(sm4_sbox_table.size() == 256);

  return sm4_sbox_table[x];
}


constexpr
uint32_t
sm4_subword(uint32_t x)
{
  auto bytes = WdRiscv::toQuarters(x);
  for (auto& q : bytes)
    q = sm4_sbox(q);
  return std::apply(WdRiscv::fromQuarters<uint8_t>, bytes);
}


constexpr
uint8_t
aes_sbox_fwd(uint8_t x)
{
  constexpr auto aes_sbox_fwd_table = std::to_array<uint8_t>(
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
    });
  static_assert(aes_sbox_fwd_table.size() == 256);

  return aes_sbox_fwd_table[x];
}


constexpr
uint8_t
aes_sbox_inv(uint8_t x)
{
  constexpr auto aes_sbox_inv_table = std::to_array<uint8_t>(
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
    });
  static_assert(aes_sbox_inv_table.size() == 256);

  return aes_sbox_inv_table[x];
}


constexpr
uint32_t
aes_subword_fwd(uint32_t x)
{
  auto bytes = WdRiscv::toQuarters(x);
  for (auto& byte : bytes)
    byte = aes_sbox_fwd(byte);
  return std::apply(WdRiscv::fromQuarters<uint8_t>, bytes);
}


constexpr
uint32_t
aes_subword_inv(uint32_t x)
{
  auto bytes = WdRiscv::toQuarters(x);
  for (auto& byte : bytes)
    byte = aes_sbox_inv(byte);
  return std::apply(WdRiscv::fromQuarters<uint8_t>, bytes);
}


constexpr
uint32_t
aes_get_column(WdRiscv::Uint128 state, uint8_t c)
{
  uint32_t res = static_cast<uint32_t>(state >> (32*c & 0x7f));
  return res;
}


constexpr
uint64_t
aes_apply_fwd_sbox_to_each_byte(uint64_t x)
{
  auto bytes = std::bit_cast<std::array<uint8_t, 8>>(x);
  for (auto& byte : bytes)
    byte = aes_sbox_fwd(byte);
  return std::bit_cast<uint64_t>(bytes);
}


constexpr
uint64_t
aes_apply_inv_sbox_to_each_byte(uint64_t x)
{
  auto bytes = std::bit_cast<std::array<uint8_t, 8>>(x);
  for (auto& byte : bytes)
    byte = aes_sbox_inv(byte);
  return std::bit_cast<uint64_t>(bytes);
}


constexpr
uint8_t
getbyte(uint64_t x, unsigned i)
{
  return std::bit_cast<std::array<uint8_t, 8>>(x)[i];
}


constexpr
uint64_t
aes_rv64_shiftrows_fwd(uint64_t rs2, uint64_t rs1)
{
  auto bytes = std::array{ getbyte(rs1, 0),
                           getbyte(rs1, 5),
                           getbyte(rs2, 2),
                           getbyte(rs2, 7),
                           getbyte(rs1, 4),
                           getbyte(rs2, 1),
                           getbyte(rs2, 6),
                           getbyte(rs1, 3) };
  return std::bit_cast<uint64_t>(bytes);
}


constexpr
uint64_t
aes_rv64_shiftrows_inv(uint64_t rs2, uint64_t rs1)
{
  auto bytes = std::array{ getbyte(rs1, 0),
                           getbyte(rs2, 5),
                           getbyte(rs2, 2),
                           getbyte(rs1, 7),
                           getbyte(rs1, 4),
                           getbyte(rs1, 1),
                           getbyte(rs2, 6),
                           getbyte(rs2, 3) };
  return std::bit_cast<uint64_t>(bytes);
}


constexpr
WdRiscv::Uint128
aes_shift_rows_fwd(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  auto ic3Bytes = toQuarters(aes_get_column(x, 3));
  auto ic2Bytes = toQuarters(aes_get_column(x, 2));
  auto ic1Bytes = toQuarters(aes_get_column(x, 1));
  auto ic0Bytes = toQuarters(aes_get_column(x, 0));

  uint32_t oc0 = fromQuarters(ic0Bytes[0], ic1Bytes[1], ic2Bytes[2], ic3Bytes[3]);
  uint32_t oc1 = fromQuarters(ic1Bytes[0], ic2Bytes[1], ic3Bytes[2], ic0Bytes[3]);
  uint32_t oc2 = fromQuarters(ic2Bytes[0], ic3Bytes[1], ic0Bytes[2], ic1Bytes[3]);
  uint32_t oc3 = fromQuarters(ic3Bytes[0], ic0Bytes[1], ic1Bytes[2], ic2Bytes[3]);

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


constexpr
WdRiscv::Uint128
aes_mixcolumns_fwd(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  uint32_t oc0 = aes_mixcolumn_fwd(aes_get_column(x, 0));
  uint32_t oc1 = aes_mixcolumn_fwd(aes_get_column(x, 1));
  uint32_t oc2 = aes_mixcolumn_fwd(aes_get_column(x, 2));
  uint32_t oc3 = aes_mixcolumn_fwd(aes_get_column(x, 3));

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


constexpr
WdRiscv::Uint128
aes_mixcolumns_inv(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  uint32_t oc0 = aes_mixcolumn_inv(aes_get_column(x, 0));
  uint32_t oc1 = aes_mixcolumn_inv(aes_get_column(x, 1));
  uint32_t oc2 = aes_mixcolumn_inv(aes_get_column(x, 2));
  uint32_t oc3 = aes_mixcolumn_inv(aes_get_column(x, 3));

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


constexpr
WdRiscv::Uint128
aes_subbytes_fwd(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  uint32_t oc0 = aes_subword_fwd(aes_get_column(x, 0));
  uint32_t oc1 = aes_subword_fwd(aes_get_column(x, 1));
  uint32_t oc2 = aes_subword_fwd(aes_get_column(x, 2));
  uint32_t oc3 = aes_subword_fwd(aes_get_column(x, 3));

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


constexpr
WdRiscv::Uint128
aes_subbytes_inv(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  uint32_t oc0 = aes_subword_inv(aes_get_column(x, 0));
  uint32_t oc1 = aes_subword_inv(aes_get_column(x, 1));
  uint32_t oc2 = aes_subword_inv(aes_get_column(x, 2));
  uint32_t oc3 = aes_subword_inv(aes_get_column(x, 3));

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


constexpr
WdRiscv::Uint128
aes_shift_rows_inv(WdRiscv::Uint128 x)
{
  using namespace WdRiscv;

  auto ic3Bytes = toQuarters(aes_get_column(x, 3));
  auto ic2Bytes = toQuarters(aes_get_column(x, 2));
  auto ic1Bytes = toQuarters(aes_get_column(x, 1));
  auto ic0Bytes = toQuarters(aes_get_column(x, 0));

  uint32_t oc0 = fromQuarters(ic0Bytes[0], ic3Bytes[1], ic2Bytes[2], ic1Bytes[3]);
  uint32_t oc1 = fromQuarters(ic1Bytes[0], ic0Bytes[1], ic3Bytes[2], ic2Bytes[3]);
  uint32_t oc2 = fromQuarters(ic2Bytes[0], ic1Bytes[1], ic0Bytes[2], ic3Bytes[3]);
  uint32_t oc3 = fromQuarters(ic3Bytes[0], ic2Bytes[1], ic1Bytes[2], ic0Bytes[3]);

  Uint128 res = fromQuarters(oc0, oc1, oc2, oc3);
  return res;
}


template<typename T>
constexpr T
bitReverse(T x)
{
  T result{0};
  unsigned bitCount = sizeof(T)*8;
  for (unsigned i = 0; i < bitCount; ++i, x >>= 1)
    result |= (x & T{1}) << (bitCount - 1 - i);
  return result;
}


template <typename T>
constexpr T brev8(const T& x)
{
  T res{0};
  for (unsigned byteIx = 0; byteIx < sizeof(x); ++byteIx)
    {
      uint8_t byte = static_cast<uint8_t>(x >> 8*byteIx);
      byte = bitReverse(byte);
      res |= T{byte} << 8*byteIx;
    }
  return res;
}


/// Rotate a word right by 8 bits.
constexpr uint32_t
aes_rotword(uint32_t x)
{
  return x >> 8 | ((x & 0xff) << 24);
}


constexpr uint32_t
sig0(uint32_t x)
{
  return ror(x, 7) ^ ror(x, 18) ^ (x >> 3);
}


constexpr uint64_t
sig0(uint64_t x)
{
  return ror(x, 1) ^ ror(x, 8) ^ (x >> 7);
}


constexpr uint32_t
sig1(uint32_t x)
{
  return ror(x, 17) ^ ror(x, 19) ^ (x >> 10);
}


constexpr uint64_t
sig1(uint64_t x)
{
  return ror(x, 19) ^ ror(x, 61) ^ (x >> 6);
}


template <typename ET, typename ET4>
constexpr void
vsha2ms(ET4& dd, const ET4& e1, const ET4& e2)
{
  using namespace WdRiscv;

  auto [w0,  w1,  w2,  w3]  = toQuarters(dd);
  auto [w4,  w9,  w10, w11] = toQuarters(e1);
  auto [w12, w13, w14, w15] = toQuarters(e2);
  ET w16 = sig1(w14) + w9  + sig0(w1) + w0;
  ET w17 = sig1(w15) + w10 + sig0(w2) + w1;
  ET w18 = sig1(w16) + w11 + sig0(w3) + w2;
  ET w19 = sig1(w17) + w12 + sig0(w4) + w3;
  dd = fromQuarters(w16, w17, w18, w19);
}


constexpr
uint32_t
sum0(uint32_t x)
{
  return ror(x, 2) ^ ror(x, 13) ^ ror(x, 22);
}


constexpr
uint64_t
sum0(uint64_t x)
{
  return ror(x, 28) ^ ror(x, 34) ^ ror(x, 39);
}


constexpr
uint32_t
sum1(uint32_t x)
{
  return ror(x, 6) ^ ror(x, 11) ^ ror(x, 25);
}


constexpr
uint64_t
sum1(uint64_t x)
{
  return ror(x, 14) ^ ror(x, 18) ^ ror(x, 41);
}


constexpr
uint32_t
ch(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) ^ ((~x) & z);
}


constexpr
uint64_t
ch(uint64_t x, uint64_t y, uint64_t z)
{
  return (x & y) ^ ((~x) & z);
}


constexpr
uint32_t
maj(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) ^ (x & z) ^ (y & z);
}


constexpr
uint64_t
maj(uint64_t x, uint64_t y, uint64_t z)
{
  return (x & y) ^ (x & z) ^ (y & z);
}


template <typename ET, typename ET4>
constexpr void
vsha2c(ET4& dd, const ET4& e1, const ET4& e2, bool high)
{
  using namespace WdRiscv;

  auto [f, e, b, a] = toQuarters(e1);
  auto [h, g, d, c] = toQuarters(dd);

  auto [m0, m1, m2, m3] = toQuarters(e2);

  ET w0{}, w1{};
  if (high)
    {
      w0 = m2;
      w1 = m3;
    }
  else
    {
      w0 = m0;
      w1 = m1;
    }
  ET t1 = h + sum1(e) + ch(e, f, g) + w0;
  ET t2 = sum0(a) + maj(a, b, c);
  h  = g;
  g  = f;
  f  = e;
  e  = d + t1;
  d  = c;
  c  = b;
  b  = a;
  a  = t1 + t2;
  t1  = h + sum1(e) + ch(e, f, g) + w1;
  t2  = sum0(a) + maj(a, b, c);
  h = g;
  g = f;
  f = e;
  e = d + t1;
  d = c;
  c = b;
  b = a;
  a = t1 + t2;
  dd = fromQuarters(f, e, b, a);
}


constexpr
uint32_t
round_key(uint32_t x, uint32_t s)
{
  return x ^ s ^ rol(s, 13) ^ rol(s, 23);
}


constexpr
uint32_t
sm4_round(uint32_t x, uint32_t s)
{
  return x ^ s ^ rol(s, 2) ^ rol(s, 10) ^ rol(s, 18) ^ rol(s, 24);
}


static constexpr
uint32_t p1(uint32_t x)
{
  return x ^ rol(x, 15) ^ rol(x, 23);
}


static constexpr
uint32_t
zvksh_w(uint32_t m16, uint32_t m9, uint32_t m3, uint32_t m13, uint32_t m6)
{
  return p1(m16 ^ m9 ^ rol(m3, 15)) ^ rol(m13, 7) ^ m6;
}


static constexpr
uint32_t
FF1(uint32_t x, uint32_t y, uint32_t z)
{
  return x ^ y ^ z;
}


static constexpr
uint32_t
FF2(uint32_t x, uint32_t y, uint32_t z)
{
  return  (x & y) | (x & z) | (y & z);
}


static constexpr
uint32_t
FF_j(uint32_t x, uint32_t y, uint32_t z, uint32_t j)
{
  return j <= 15 ? FF1(x, y, z) : FF2(x, y, z);
}


static constexpr
uint32_t
GG1(uint32_t x, uint32_t y, uint32_t z)
{
  return x ^ y ^ z;
}


static constexpr
uint32_t
GG2(uint32_t x, uint32_t y, uint32_t z)
{
  return (x & y) | (~x & z);
}


constexpr
uint32_t
GG_j(uint32_t x, uint32_t y, uint32_t z, uint32_t j)
{
  return j <= 15 ? GG1(x, y, z) : GG2(x, y, z);
}


constexpr
uint32_t
T_j(uint32_t j)
{
  return j <= 15 ? 0x79CC4519 : 0x7A879D8A;
}


constexpr
uint32_t
P_0(uint32_t x)
{
  return x ^ rol(x,  9) ^ rol(x, 17);
}
