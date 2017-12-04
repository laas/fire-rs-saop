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

#ifndef IMC_POWEROPERATION_HPP_INCLUDED_
#define IMC_POWEROPERATION_HPP_INCLUDED_

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
  //! Power Operation.
  class PowerOperation: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Power Down.
      POP_PWR_DOWN = 0,
      //! Power Down in Progress.
      POP_PWR_DOWN_IP = 1,
      //! Power Down Aborted.
      POP_PWR_DOWN_ABORTED = 2,
      //! Schedule Power Down.
      POP_SCHED_PWR_DOWN = 3,
      //! Power Up.
      POP_PWR_UP = 4,
      //! Power Up in Progress.
      POP_PWR_UP_IP = 5,
      //! Schedule Power Up.
      POP_SCHED_PWR_UP = 6
    };

    //! Operation.
    uint8_t op;
    //! Time Remaining.
    float time_remain;
    //! Scheduled Time.
    double sched_time;

    static uint16_t
    getIdStatic(void)
    {
      return 308;
    }

    static PowerOperation*
    cast(Message* msg__)
    {
      return (PowerOperation*)msg__;
    }

    PowerOperation(void)
    {
      m_header.mgid = PowerOperation::getIdStatic();
      clear();
    }

    PowerOperation*
    clone(void) const
    {
      return new PowerOperation(*this);
    }

    void
    clear(void)
    {
      op = 0;
      time_remain = 0;
      sched_time = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PowerOperation& other__ = static_cast<const PowerOperation&>(msg__);
      if (op != other__.op) return false;
      if (time_remain != other__.time_remain) return false;
      if (sched_time != other__.sched_time) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(time_remain, ptr__);
      ptr__ += IMC::serialize(sched_time, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(time_remain, bfr__, size__);
      bfr__ += IMC::deserialize(sched_time, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_remain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sched_time, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PowerOperation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PowerOperation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 13;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "time_remain", time_remain, nindent__);
      IMC::toJSON(os__, "sched_time", sched_time, nindent__);
    }
  };
}

#endif
