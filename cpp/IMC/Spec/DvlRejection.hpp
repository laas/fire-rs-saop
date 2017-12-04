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

#ifndef IMC_DVLREJECTION_HPP_INCLUDED_
#define IMC_DVLREJECTION_HPP_INCLUDED_

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
  //! DVL Rejection.
  class DvlRejection: public Message
  {
  public:
    //! Reason.
    enum ReasonEnum
    {
      //! Innovation Threshold - X.
      RR_INNOV_THRESHOLD_X = 0,
      //! Innovation Threshold - Y.
      RR_INNOV_THRESHOLD_Y = 1,
      //! Absolute Threshold - X.
      RR_ABS_THRESHOLD_X = 2,
      //! Absolute Threshold - Y.
      RR_ABS_THRESHOLD_Y = 3
    };

    //! Type of velocity.
    enum TypeofvelocityBits
    {
      //! Ground velocity.
      TYPE_GV = 0x01,
      //! Water velocity.
      TYPE_WV = 0x02
    };

    //! Type of velocity.
    uint8_t type;
    //! Reason.
    uint8_t reason;
    //! Value.
    float value;
    //! Timestep.
    float timestep;

    static uint16_t
    getIdStatic(void)
    {
      return 358;
    }

    static DvlRejection*
    cast(Message* msg__)
    {
      return (DvlRejection*)msg__;
    }

    DvlRejection(void)
    {
      m_header.mgid = DvlRejection::getIdStatic();
      clear();
    }

    DvlRejection*
    clone(void) const
    {
      return new DvlRejection(*this);
    }

    void
    clear(void)
    {
      type = 0;
      reason = 0;
      value = 0;
      timestep = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DvlRejection& other__ = static_cast<const DvlRejection&>(msg__);
      if (type != other__.type) return false;
      if (reason != other__.reason) return false;
      if (value != other__.value) return false;
      if (timestep != other__.timestep) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(reason, ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      ptr__ += IMC::serialize(timestep, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(reason, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(timestep, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(reason, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(value, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timestep, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DvlRejection::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DvlRejection";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 10;
    }

    double
    getValueFP(void) const
    {
      return static_cast<double>(value);
    }

    void
    setValueFP(double val)
    {
      value = static_cast<float>(val);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "reason", reason, nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
      IMC::toJSON(os__, "timestep", timestep, nindent__);
    }
  };
}

#endif
