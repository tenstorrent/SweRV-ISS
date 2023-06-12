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

#include <algorithm>
#include <bit>
#include "util.hpp"
#include "WhisperMessage.h"


// These can be replaced with std::spanstream in C++23
class spanstream
{
public:
  constexpr spanstream(std::span<char> data) : data_(data)
  { }

  constexpr std::size_t tellg() const
  { return pos_; }

protected:
  std::span<char> data_;
  std::size_t pos_ = 0;
};

class ispanstream: public spanstream
{
public:
  using spanstream::spanstream;

  template <typename T>
  requires requires (T t) { std::span(t); }
  constexpr ispanstream& operator>>(T&& value)
  {
    auto valueBytes = std::as_writable_bytes(std::span(std::forward<T>(value)));
    auto thisBytes  = std::as_bytes(data_.subspan(pos_));

    assert(valueBytes.size() <= thisBytes.size());

    std::copy_n(thisBytes.begin(), valueBytes.size(), valueBytes.begin());
    pos_ += valueBytes.size();
    return *this;
  }

  template <std::integral T>
  constexpr ispanstream& operator>>(T& value)
  {
    return *this >> std::span<T, 1>(&value, 1);
  }
};

class ospanstream : public spanstream
{
public:
  using spanstream::spanstream;

  template <typename T>
  requires requires (T t) { std::span(t); }
  constexpr ospanstream& operator<<(T&& value)
  {
    auto valueBytes = std::as_bytes(std::span(std::forward<T>(value)));
    auto thisBytes  = std::as_writable_bytes(data_.subspan(pos_));

    assert(valueBytes.size() <= thisBytes.size());

    std::copy(valueBytes.begin(), valueBytes.end(), thisBytes.begin());
    pos_ += valueBytes.size();
    return *this;
  }

  template <std::integral T>
  constexpr ospanstream& operator<<(const T& value)
  {
    return *this << std::span<const T, 1>(&value, 1);
  }
};


template <std::integral T>
class hton
{
public:
  constexpr hton(T& v) : val_(&v)
  { }

  friend constexpr ospanstream& operator<<(ospanstream& s, hton h)
  {
    typename std::remove_const<T>::type v = *h.val_;
    if constexpr (std::endian::native == std::endian::little)
      v = util::byteswap(v);
    s << v;
    return s;
  }

  friend constexpr ispanstream& operator>>(ispanstream& s, hton h)
  {
    T v;
    s >> v;
    if constexpr (std::endian::native == std::endian::little)
      v = util::byteswap(v);
    *h.val_ = v;
    return s;
  }

private:
  T* val_;
};

template <typename T>
auto ntoh(T&& v)
{ return hton(std::forward<T>(v)); }


WhisperMessage
WhisperMessage::deserializeFrom(std::span<char> buffer)
{
  WhisperMessage msg;
  assert (buffer.size() >= sizeof(msg));

  ispanstream sstream(buffer);

  sstream >> ntoh(msg.hart)
          >> ntoh(msg.type)
          >> ntoh(msg.resource)
          >> ntoh(msg.size)
          >> ntoh(msg.flags)
          >> ntoh(msg.instrTag)
          >> ntoh(msg.time)
          >> ntoh(msg.address)
          >> ntoh(msg.value)
          >> msg.buffer
          >> msg.tag;

  return msg;
}


size_t
WhisperMessage::serializeTo(std::span<char> buffer) const
{
  assert (buffer.size() >= sizeof(*this));

  ospanstream sstream(buffer);

  sstream << hton(this->hart)
          << hton(this->type)
          << hton(this->resource)
          << hton(this->size)
          << hton(this->flags)
          << hton(this->instrTag)
          << hton(this->time)
          << hton(this->address)
          << hton(this->value)
          << this->buffer
          << this->tag;

  std::size_t writtenSize = sstream.tellg();
  if (buffer.size() > writtenSize)
    {
      std::fill(std::next(buffer.begin(),
                          static_cast<std::ptrdiff_t>(writtenSize)),
                buffer.end(),
                0);
    }

  return sizeof(*this);
}
