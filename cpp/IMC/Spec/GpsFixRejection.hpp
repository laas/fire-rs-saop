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

#ifndef IMC_GPSFIXREJECTION_HPP_INCLUDED_
#define IMC_GPSFIXREJECTION_HPP_INCLUDED_

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
  //! GPS Fix Rejection.
  class GpsFixRejection: public Message
  {
  public:
    //! Reason.
    enum ReasonEnum
    {
      //! Above Threshold.
      RR_ABOVE_THRESHOLD = 0,
      //! Invalid Fix.
      RR_INVALID = 1,
      //! Above Maximum HDOP.
      RR_ABOVE_MAX_HDOP = 2,
      //! Above Maximum HACC.
      RR_ABOVE_MAX_HACC = 3,
      //! Lost Validity Bit.
      RR_LOST_VAL_BIT = 4
    };

    //! UTC Time of Fix.
    float utc_time;
    //! Reason.
    uint8_t reason;

    static uint16_t
    getIdStatic(void)
    {
      return 356;
    }

    static GpsFixRejection*
    cast(Message* msg__)
    {
      return (GpsFixRejection*)msg__;
    }

    GpsFixRejection(void)
    {
      m_header.mgid = GpsFixRejection::getIdStatic();
      clear();
    }

    GpsFixRejection*
    clone(void) const
    {
      return new GpsFixRejection(*this);
    }

    void
    clear(void)
    {
      utc_time = 0;
      reason = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GpsFixRejection& other__ = static_cast<const GpsFixRejection&>(msg__);
      if (utc_time != other__.utc_time) return false;
      if (reason != other__.reason) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(utc_time, ptr__);
      ptr__ += IMC::serialize(reason, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(utc_time, bfr__, size__);
      bfr__ += IMC::deserialize(reason, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(utc_time, bfr__, size__);
      bfr__ += IMC::deserialize(reason, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GpsFixRejection::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GpsFixRejection";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "utc_time", utc_time, nindent__);
      IMC::toJSON(os__, "reason", reason, nindent__);
    }
  };
}

#endif
