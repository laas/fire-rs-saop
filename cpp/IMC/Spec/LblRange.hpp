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

#ifndef IMC_LBLRANGE_HPP_INCLUDED_
#define IMC_LBLRANGE_HPP_INCLUDED_

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
  //! LBL Range.
  class LblRange: public Message
  {
  public:
    //! Beacon Identification Number.
    uint8_t id;
    //! Range.
    float range;

    static uint16_t
    getIdStatic(void)
    {
      return 200;
    }

    static LblRange*
    cast(Message* msg__)
    {
      return (LblRange*)msg__;
    }

    LblRange(void)
    {
      m_header.mgid = LblRange::getIdStatic();
      clear();
    }

    LblRange*
    clone(void) const
    {
      return new LblRange(*this);
    }

    void
    clear(void)
    {
      id = 0;
      range = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LblRange& other__ = static_cast<const LblRange&>(msg__);
      if (id != other__.id) return false;
      if (range != other__.range) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(range, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(range, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(range, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LblRange::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LblRange";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    uint16_t
    getSubId(void) const
    {
      return id;
    }

    void
    setSubId(uint16_t subid)
    {
      id = (uint8_t)subid;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      IMC::toJSON(os__, "range", range, nindent__);
    }
  };
}

#endif
