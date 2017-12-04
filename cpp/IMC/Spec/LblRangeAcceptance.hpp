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

#ifndef IMC_LBLRANGEACCEPTANCE_HPP_INCLUDED_
#define IMC_LBLRANGEACCEPTANCE_HPP_INCLUDED_

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
  //! LBL Range Acceptance.
  class LblRangeAcceptance: public Message
  {
  public:
    //! Acceptance.
    enum AcceptanceEnum
    {
      //! Accepted.
      RR_ACCEPTED = 0,
      //! Rejected - Above Threshold.
      RR_ABOVE_THRESHOLD = 1,
      //! Rejected - Singular Point.
      RR_SINGULAR = 2,
      //! Rejected - Not Enough Information.
      RR_NO_INFO = 3,
      //! Rejected - Vehicle At Surface.
      RR_AT_SURFACE = 4
    };

    //! Beacon Identification Number.
    uint8_t id;
    //! Range.
    float range;
    //! Acceptance.
    uint8_t acceptance;

    static uint16_t
    getIdStatic(void)
    {
      return 357;
    }

    static LblRangeAcceptance*
    cast(Message* msg__)
    {
      return (LblRangeAcceptance*)msg__;
    }

    LblRangeAcceptance(void)
    {
      m_header.mgid = LblRangeAcceptance::getIdStatic();
      clear();
    }

    LblRangeAcceptance*
    clone(void) const
    {
      return new LblRangeAcceptance(*this);
    }

    void
    clear(void)
    {
      id = 0;
      range = 0;
      acceptance = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LblRangeAcceptance& other__ = static_cast<const LblRangeAcceptance&>(msg__);
      if (id != other__.id) return false;
      if (range != other__.range) return false;
      if (acceptance != other__.acceptance) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(range, ptr__);
      ptr__ += IMC::serialize(acceptance, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(range, bfr__, size__);
      bfr__ += IMC::deserialize(acceptance, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(range, bfr__, size__);
      bfr__ += IMC::deserialize(acceptance, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LblRangeAcceptance::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LblRangeAcceptance";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
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
      IMC::toJSON(os__, "acceptance", acceptance, nindent__);
    }
  };
}

#endif
