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

#ifndef IMC_LBLESTIMATE_HPP_INCLUDED_
#define IMC_LBLESTIMATE_HPP_INCLUDED_

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
#include "../Spec/LblBeacon.hpp"

namespace IMC
{
  //! LBL Beacon Position Estimate.
  class LblEstimate: public Message
  {
  public:
    //! LBL Beacon Configuration.
    InlineMessage<LblBeacon> beacon;
    //! North position.
    float x;
    //! East position.
    float y;
    //! North position variance.
    float var_x;
    //! East position variance.
    float var_y;
    //! Distance.
    float distance;

    static uint16_t
    getIdStatic(void)
    {
      return 360;
    }

    static LblEstimate*
    cast(Message* msg__)
    {
      return (LblEstimate*)msg__;
    }

    LblEstimate(void)
    {
      m_header.mgid = LblEstimate::getIdStatic();
      clear();
      beacon.setParent(this);
    }

    LblEstimate*
    clone(void) const
    {
      return new LblEstimate(*this);
    }

    void
    clear(void)
    {
      beacon.clear();
      x = 0;
      y = 0;
      var_x = 0;
      var_y = 0;
      distance = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LblEstimate& other__ = static_cast<const LblEstimate&>(msg__);
      if (beacon != other__.beacon) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (var_x != other__.var_x) return false;
      if (var_y != other__.var_y) return false;
      if (distance != other__.distance) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += beacon.serialize(ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(var_x, ptr__);
      ptr__ += IMC::serialize(var_y, ptr__);
      ptr__ += IMC::serialize(distance, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += beacon.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(var_x, bfr__, size__);
      bfr__ += IMC::deserialize(var_y, bfr__, size__);
      bfr__ += IMC::deserialize(distance, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += beacon.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(var_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(var_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(distance, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LblEstimate::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LblEstimate";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 20;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return beacon.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      beacon.toJSON(os__, "beacon", nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "var_x", var_x, nindent__);
      IMC::toJSON(os__, "var_y", var_y, nindent__);
      IMC::toJSON(os__, "distance", distance, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!beacon.isNull())
      {
        beacon.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!beacon.isNull())
      {
        beacon.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!beacon.isNull())
      {
        beacon.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!beacon.isNull())
      {
        beacon.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!beacon.isNull())
      {
        beacon.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
