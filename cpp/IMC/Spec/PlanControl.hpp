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

#ifndef IMC_PLANCONTROL_HPP_INCLUDED_
#define IMC_PLANCONTROL_HPP_INCLUDED_

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
  //! Plan Control.
  class PlanControl: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Request.
      PC_REQUEST = 0,
      //! Reply -- Success.
      PC_SUCCESS = 1,
      //! Reply -- Failure.
      PC_FAILURE = 2,
      //! Reply -- In Progress.
      PC_IN_PROGRESS = 3
    };

    //! Operation.
    enum OperationEnum
    {
      //! Start Plan.
      PC_START = 0,
      //! Stop Plan.
      PC_STOP = 1,
      //! Load Plan.
      PC_LOAD = 2,
      //! Get Plan.
      PC_GET = 3
    };

    //! Flags.
    enum FlagsBits
    {
      //! Calibrate Vehicle.
      FLG_CALIBRATE = 0x0001,
      //! Ignore Errors.
      FLG_IGNORE_ERRORS = 0x0002
    };

    //! Type.
    uint8_t type;
    //! Operation.
    uint8_t op;
    //! Request ID.
    uint16_t request_id;
    //! Plan Identifier.
    std::string plan_id;
    //! Flags.
    uint16_t flags;
    //! Request/Reply Argument.
    InlineMessage<Message> arg;
    //! Complementary Info.
    std::string info;

    static uint16_t
    getIdStatic(void)
    {
      return 559;
    }

    static PlanControl*
    cast(Message* msg__)
    {
      return (PlanControl*)msg__;
    }

    PlanControl(void)
    {
      m_header.mgid = PlanControl::getIdStatic();
      clear();
      arg.setParent(this);
    }

    PlanControl*
    clone(void) const
    {
      return new PlanControl(*this);
    }

    void
    clear(void)
    {
      type = 0;
      op = 0;
      request_id = 0;
      plan_id.clear();
      flags = 0;
      arg.clear();
      info.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanControl& other__ = static_cast<const PlanControl&>(msg__);
      if (type != other__.type) return false;
      if (op != other__.op) return false;
      if (request_id != other__.request_id) return false;
      if (plan_id != other__.plan_id) return false;
      if (flags != other__.flags) return false;
      if (arg != other__.arg) return false;
      if (info != other__.info) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(request_id, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += arg.serialize(ptr__);
      ptr__ += IMC::serialize(info, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(request_id, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += arg.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(request_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(flags, bfr__, size__);
      bfr__ += arg.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + arg.getSerializationSize() + IMC::getSerializationSize(info);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "request_id", request_id, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
      arg.toJSON(os__, "arg", nindent__);
      IMC::toJSON(os__, "info", info, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!arg.isNull())
      {
        arg.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!arg.isNull())
      {
        arg.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!arg.isNull())
      {
        arg.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!arg.isNull())
      {
        arg.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!arg.isNull())
      {
        arg.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
