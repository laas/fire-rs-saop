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

#ifndef IMC_ACOUSTICOPERATION_HPP_INCLUDED_
#define IMC_ACOUSTICOPERATION_HPP_INCLUDED_

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
  //! Acoustic Operation.
  class AcousticOperation: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Abort.
      AOP_ABORT = 0,
      //! Abort in Progress.
      AOP_ABORT_IP = 1,
      //! Abort Timeout.
      AOP_ABORT_TIMEOUT = 2,
      //! Abort Acknowledged.
      AOP_ABORT_ACKED = 3,
      //! Range Request.
      AOP_RANGE = 4,
      //! Range in Progress.
      AOP_RANGE_IP = 5,
      //! Range Timeout.
      AOP_RANGE_TIMEOUT = 6,
      //! Range Received.
      AOP_RANGE_RECVED = 7,
      //! Modem is Busy.
      AOP_BUSY = 8,
      //! Unsupported operation.
      AOP_UNSUPPORTED = 9,
      //! Transducer Not Detected.
      AOP_NO_TXD = 10,
      //! Send Message.
      AOP_MSG = 11,
      //! Message Send -- Queued.
      AOP_MSG_QUEUED = 12,
      //! Message Send -- In progress.
      AOP_MSG_IP = 13,
      //! Message Send -- Done.
      AOP_MSG_DONE = 14,
      //! Message Send -- Failure.
      AOP_MSG_FAILURE = 15,
      //! Send Short Message.
      AOP_MSG_SHORT = 16,
      //! Initiate Reverse Range.
      AOP_REVERSE_RANGE = 17
    };

    //! Operation.
    uint8_t op;
    //! System.
    std::string system;
    //! Range.
    float range;
    //! Message To Send.
    InlineMessage<Message> msg;

    static uint16_t
    getIdStatic(void)
    {
      return 211;
    }

    static AcousticOperation*
    cast(Message* msg__)
    {
      return (AcousticOperation*)msg__;
    }

    AcousticOperation(void)
    {
      m_header.mgid = AcousticOperation::getIdStatic();
      clear();
      msg.setParent(this);
    }

    AcousticOperation*
    clone(void) const
    {
      return new AcousticOperation(*this);
    }

    void
    clear(void)
    {
      op = 0;
      system.clear();
      range = 0;
      msg.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AcousticOperation& other__ = static_cast<const AcousticOperation&>(msg__);
      if (op != other__.op) return false;
      if (system != other__.system) return false;
      if (range != other__.range) return false;
      if (msg != other__.msg) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(system, ptr__);
      ptr__ += IMC::serialize(range, ptr__);
      ptr__ += msg.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(system, bfr__, size__);
      bfr__ += IMC::deserialize(range, bfr__, size__);
      bfr__ += msg.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(system, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(range, bfr__, size__);
      bfr__ += msg.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AcousticOperation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AcousticOperation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(system) + msg.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "system", system, nindent__);
      IMC::toJSON(os__, "range", range, nindent__);
      msg.toJSON(os__, "msg", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!msg.isNull())
      {
        msg.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!msg.isNull())
      {
        msg.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!msg.isNull())
      {
        msg.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!msg.isNull())
      {
        msg.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!msg.isNull())
      {
        msg.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
