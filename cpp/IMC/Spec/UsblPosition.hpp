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
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 4d8734a1111656aac56f803acdc90c22                            *
//***************************************************************************

#ifndef IMC_USBLPOSITION_HPP_INCLUDED_
#define IMC_USBLPOSITION_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include "../Base/Config.hpp"
#include "../Base/Message.hpp"
#include "../Base/InlineMessage.hpp"
#include "../Base/MessageList.hpp"
#include "../Base/JSON.hpp"
#include "../Base/Serialization.hpp"
#include "../Spec/Enumerations.hpp"
#include "../Spec/Bitfields.hpp"

namespace IMC
{
  //! USBL Position.
  class UsblPosition: public Message
  {
  public:
    //! Target.
    uint16_t target;
    //! X.
    float x;
    //! Y.
    float y;
    //! Z.
    float z;

    static uint16_t
    getIdStatic(void)
    {
      return 891;
    }

    static UsblPosition*
    cast(Message* msg__)
    {
      return (UsblPosition*)msg__;
    }

    UsblPosition(void)
    {
      m_header.mgid = UsblPosition::getIdStatic();
      clear();
    }

    UsblPosition*
    clone(void) const
    {
      return new UsblPosition(*this);
    }

    void
    clear(void)
    {
      target = 0;
      x = 0;
      y = 0;
      z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblPosition& other__ = static_cast<const UsblPosition&>(msg__);
      if (target != other__.target) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(target, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(target, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(target, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblPosition::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblPosition";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 14;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "target", target, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
    }
  };
}

#endif
