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

#ifndef IMC_POWERCHANNELCONTROL_HPP_INCLUDED_
#define IMC_POWERCHANNELCONTROL_HPP_INCLUDED_

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
  //! Power Channel Control.
  class PowerChannelControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Turn Off.
      PCC_OP_TURN_OFF = 0,
      //! Turn On.
      PCC_OP_TURN_ON = 1,
      //! Toggle.
      PCC_OP_TOGGLE = 2,
      //! Schedule Turn On.
      PCC_OP_SCHED_ON = 3,
      //! Schedule Turn Off.
      PCC_OP_SCHED_OFF = 4,
      //! Reset Schedules.
      PCC_OP_SCHED_RESET = 5,
      //! Save Current State.
      PCC_OP_SAVE = 6
    };

    //! Channel Name.
    std::string name;
    //! Operation.
    uint8_t op;
    //! Scheduled Time.
    double sched_time;

    static uint16_t
    getIdStatic(void)
    {
      return 309;
    }

    static PowerChannelControl*
    cast(Message* msg__)
    {
      return (PowerChannelControl*)msg__;
    }

    PowerChannelControl(void)
    {
      m_header.mgid = PowerChannelControl::getIdStatic();
      clear();
    }

    PowerChannelControl*
    clone(void) const
    {
      return new PowerChannelControl(*this);
    }

    void
    clear(void)
    {
      name.clear();
      op = 0;
      sched_time = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PowerChannelControl& other__ = static_cast<const PowerChannelControl&>(msg__);
      if (name != other__.name) return false;
      if (op != other__.op) return false;
      if (sched_time != other__.sched_time) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(name, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(sched_time, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(sched_time, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sched_time, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PowerChannelControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PowerChannelControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "name", name, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "sched_time", sched_time, nindent__);
    }
  };
}

#endif
