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

#ifndef IMC_PLANGENERATION_HPP_INCLUDED_
#define IMC_PLANGENERATION_HPP_INCLUDED_

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
  //! Plan Generation.
  class PlanGeneration: public Message
  {
  public:
    //! Command.
    enum CommandEnum
    {
      //! Generate.
      CMD_GENERATE = 0,
      //! Execute.
      CMD_EXECUTE = 1
    };

    //! Operation.
    enum OperationEnum
    {
      //! Request.
      OP_REQUEST = 0,
      //! Error.
      OP_ERROR = 1,
      //! Success.
      OP_SUCCESS = 2
    };

    //! Command.
    uint8_t cmd;
    //! Operation.
    uint8_t op;
    //! Plan Identifier.
    std::string plan_id;
    //! Parameters.
    std::string params;

    static uint16_t
    getIdStatic(void)
    {
      return 562;
    }

    static PlanGeneration*
    cast(Message* msg__)
    {
      return (PlanGeneration*)msg__;
    }

    PlanGeneration(void)
    {
      m_header.mgid = PlanGeneration::getIdStatic();
      clear();
    }

    PlanGeneration*
    clone(void) const
    {
      return new PlanGeneration(*this);
    }

    void
    clear(void)
    {
      cmd = 0;
      op = 0;
      plan_id.clear();
      params.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanGeneration& other__ = static_cast<const PlanGeneration&>(msg__);
      if (cmd != other__.cmd) return false;
      if (op != other__.op) return false;
      if (plan_id != other__.plan_id) return false;
      if (params != other__.params) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(cmd, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(params, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(cmd, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(params, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(cmd, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(params, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanGeneration::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanGeneration";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(params);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "cmd", cmd, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "params", params, nindent__);
    }
  };
}

#endif
