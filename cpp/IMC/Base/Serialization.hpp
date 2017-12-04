//***************************************************************************
// Copyright 2017 OceanScan - Marine Systems & Technology, Lda.             *
//***************************************************************************
// Licensed under the Apache License, Version 2.0 (the "License");          *
// you may not use this file except in compliance with the License.         *
// You may obtain a copy of the License at                                  *
//                                                                          *
// http://www.apache.org/licenses/LICENSE-2.0                               *
//                                                                          *
// Unless required by applicable law or agreed to in writing, software      *
// distributed under the License is distributed on an "AS IS" BASIS,        *
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
// See the License for the specific language governing permissions and      *
// limitations under the License.                                           *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

#ifndef IMC_SERIALIZATION_HPP_INCLUDED_
#define IMC_SERIALIZATION_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <istream>
#include <vector>
#include <string>
#include <ostream>
#include <istream>

// IMC Base headers.
#include "ByteCopy.hpp"
#include "Exceptions.hpp"
#include "Message.hpp"
#include "Factory.hpp"
#include "CRC16.hpp"

namespace IMC
{
  //! Retrieve the number of bytes required to serialize a variable
  //! of type 'plaintext'.
  //! @param[in] variable variable.
  //! @return number of bytes required to serialize variable.
  inline size_t
  getSerializationSize(const std::string &variable)
  {
    return variable.size() + 2;
  }

  //! Retrieve the number of bytes required to serialize a variable
  //! of type 'rawdata'.
  //! @param[in] variable variable.
  //! @return number of bytes required to serialize variable.
  inline size_t
  getSerializationSize(const std::vector<char> &variable)
  {
    return variable.size() + 2;
  }

  //! Serializer for scalar types.
  //! @param t scalar to serialize.
  //! @param bfr buffer where to place the serialized bytes.
  //! @return number of serialized bytes.
  template<typename Type>
  inline size_t
  serialize(const Type t, uint8_t *bfr)
  {
    uint16_t size = sizeof(Type);
    std::memcpy(bfr, &t, size);
    return size;
  }

  inline size_t
  serialize(const std::string &t, uint8_t *bfr)
  {
    const uint16_t s = static_cast<uint16_t>(t.size());
    std::memcpy(bfr, &s, sizeof(s));
    bfr += sizeof(s);
    std::memcpy(bfr, t.c_str(), s);
    return s + 2;
  }

  //! Deserializer for string objects.
  //! @param t string object where to place the deserialized bytes.
  //! @param bfr buffer where to read the serialized bytes.
  //! @param length amount of bytes available to deserialize.
  //! @return number of deserialized bytes.
  //! @throw BufferTooShort
  inline size_t
  deserialize(std::string &t, const uint8_t *bfr, size_t &bfr_len)
  {
    if (bfr_len < 2)
      throw BufferTooShort();

    uint16_t s = 0;
    std::memcpy(&s, bfr, 2);

    if (bfr_len < (unsigned) (s + 2))
      throw BufferTooShort();

    t.assign((const char *) (bfr + 2), s);
    bfr_len -= s + 2;

    return s + 2;
  }

  inline size_t
  reverseDeserialize(std::string &t, const uint8_t *bfr, size_t &bfr_len)
  {
    if (bfr_len < 2)
      throw BufferTooShort();

    uint16_t s = 0;
    ByteCopy::rcopy(s, bfr);

    if (bfr_len < (unsigned) (s + 2))
      throw BufferTooShort();

    t.assign((const char *) (bfr + 2), s);
    bfr_len -= s + 2;

    return s + 2;
  }

  inline size_t
  serialize(const std::vector<char> &t, uint8_t *bfr)
  {
    const uint16_t s = static_cast<uint16_t>(t.size());
    std::memcpy(bfr, &s, sizeof(s));
    bfr += sizeof(s);
    if (s > 0)
      std::memcpy(bfr, &t[0], s);
    return s + 2;
  }

  inline size_t
  deserialize(std::vector<char> &t, const uint8_t *bfr, size_t &bfr_len)
  {
    if (bfr_len < 2)
      throw BufferTooShort();

    uint16_t s = 0;
    std::memcpy(&s, bfr, 2);

    if (bfr_len < (unsigned) (s + 2))
      throw BufferTooShort();

    t.assign((const char *) (bfr + 2), (const char *) (bfr + 2 + s));
    bfr_len -= s + 2;

    return s + 2;
  }

  static inline size_t
  reverseDeserialize(std::vector<char> &t, const uint8_t *bfr, size_t &bfr_len)
  {
    if (bfr_len < 2)
      throw BufferTooShort();

    uint16_t s = 0;
    ByteCopy::rcopy(s, bfr);

    if (bfr_len < (unsigned) (s + 2))
      throw BufferTooShort();

    t.assign((const char *) (bfr + 2), (const char *) (bfr + 2 + s));
    bfr_len -= s + 2;

    return s + 2;
  }

  //! Deserializer for scalar types.
  //! @param t scalar where to place the unserialized bytes.
  //! @param bfr buffer where to read the serialized bytes.
  //! @param length amount of bytes available to unserialize.
  //! @return number of serialized bytes.
  //! @throw BufferTooShort
  template<typename Type>
  inline size_t
  deserialize(Type &t, const uint8_t *bfr, size_t &length)
  {
    uint16_t size = sizeof(Type);

    if (length < size)
      throw BufferTooShort();

    std::memcpy(&t, bfr, size);
    length -= size;

    return size;
  }

  //! Deserialize a numeric field with a different byte.
  //! @param t variable where to place the unserialized result.
  //! @param bfr buffer where to read the serialized bytes.
  //! @param length amount of bytes available to unserialize.
  //! @return number of serialized bytes.
  //! @throw BufferTooShort
  template<typename Type>
  inline size_t
  reverseDeserialize(Type &t, const uint8_t *bfr, size_t &length)
  {
    uint16_t size = sizeof(Type);

    if (length < size)
      throw BufferTooShort();

    ByteCopy::rcopy(t, bfr);
    length -= size;

    return size;
  }
}

#endif
