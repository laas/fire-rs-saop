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

#ifndef IMC_CLOCKCONTROL_HPP_INCLUDED_
#define IMC_CLOCKCONTROL_HPP_INCLUDED_

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
  //! Clock Control.
  class ClockControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Execute Sync..
      COP_SYNC_EXEC = 0,
      //! Request Sync..
      COP_SYNC_REQUEST = 1,
      //! Sync. Started.
      COP_SYNC_STARTED = 2,
      //! Sync. done.
      COP_SYNC_DONE = 3,
      //! Set Timezone .
      COP_SET_TZ = 4,
      //! Timezone Setup.
      COP_SET_TZ_DONE = 5
    };

    //! Operation.
    uint8_t op;
    //! Clock.
    double clock;
    //! Timezone.
    int8_t tz;

    static uint16_t
    getIdStatic(void)
    {
      return 106;
    }

    static ClockControl*
    cast(Message* msg__)
    {
      return (ClockControl*)msg__;
    }

    ClockControl(void)
    {
      m_header.mgid = ClockControl::getIdStatic();
      clear();
    }

    ClockControl*
    clone(void) const
    {
      return new ClockControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      clock = 0;
      tz = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ClockControl& other__ = static_cast<const ClockControl&>(msg__);
      if (op != other__.op) return false;
      if (clock != other__.clock) return false;
      if (tz != other__.tz) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(clock, ptr__);
      ptr__ += IMC::serialize(tz, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(clock, bfr__, size__);
      bfr__ += IMC::deserialize(tz, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(clock, bfr__, size__);
      bfr__ += IMC::deserialize(tz, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ClockControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ClockControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 10;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "clock", clock, nindent__);
      IMC::toJSON(os__, "tz", tz, nindent__);
    }
  };
}

#endif
