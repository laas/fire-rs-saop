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

#ifndef IMC_SMSTX_HPP_INCLUDED_
#define IMC_SMSTX_HPP_INCLUDED_

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
  //! SMS Transmit.
  class SmsTx: public Message
  {
  public:
    //! Sequence Number.
    uint32_t seq;
    //! Destination.
    std::string destination;
    //! Timeout.
    uint16_t timeout;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 157;
    }

    static SmsTx*
    cast(Message* msg__)
    {
      return (SmsTx*)msg__;
    }

    SmsTx(void)
    {
      m_header.mgid = SmsTx::getIdStatic();
      clear();
    }

    SmsTx*
    clone(void) const
    {
      return new SmsTx(*this);
    }

    void
    clear(void)
    {
      seq = 0;
      destination.clear();
      timeout = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SmsTx& other__ = static_cast<const SmsTx&>(msg__);
      if (seq != other__.seq) return false;
      if (destination != other__.destination) return false;
      if (timeout != other__.timeout) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(seq, ptr__);
      ptr__ += IMC::serialize(destination, ptr__);
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(destination, bfr__, size__);
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(seq, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(destination, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SmsTx::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SmsTx";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(destination) + IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "seq", seq, nindent__);
      IMC::toJSON(os__, "destination", destination, nindent__);
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
