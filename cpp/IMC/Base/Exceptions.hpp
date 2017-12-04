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

#ifndef IMC_EXCEPTIONS_HPP_INCLUDED_
#define IMC_EXCEPTIONS_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <stdexcept>

// IMC Base headers.
#include "String.hpp"

namespace IMC
{
  //! Invalid synchronization number exception.
  class InvalidSync: public std::runtime_error
  {
  public:
    explicit
    InvalidSync(uint16_t sync):
      std::runtime_error("invalid synchronization number: " + String::toHex(sync))
    { }
  };

  //! Invalid message identification number exception.
  class InvalidMessageId: public std::runtime_error
  {
  public:
    explicit
    InvalidMessageId(uint32_t id):
      std::runtime_error("invalid message identification number: " + String::str(id))
    { }
  };

  //! Invalid message abbreviation exception.
  class InvalidMessageAbbrev: public std::runtime_error
  {
  public:
    explicit
    InvalidMessageAbbrev(const std::string& abbrev):
      std::runtime_error("invalid message abbreviation: " + abbrev)
    { }
  };

  //! Invalid CRC exception.
  class InvalidCrc: public std::runtime_error
  {
  public:
    InvalidCrc():
      std::runtime_error("invalid CRC")
    { }
  };

  //! Buffer too short to be unpacked exception.
  class BufferTooShort: public std::runtime_error
  {
  public:
    BufferTooShort(void):
      std::runtime_error("buffer is too short to be unpacked")
    { }
  };

  class InvalidMessageSize: public std::runtime_error
  {
  public:
    explicit
    InvalidMessageSize(size_t size):
      std::runtime_error(String::str("invalid message size %u", size))
    { }
  };
}

#endif
