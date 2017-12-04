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

#ifndef IMC_TREXOPERATION_HPP_INCLUDED_
#define IMC_TREXOPERATION_HPP_INCLUDED_

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
#include "../Spec/TrexToken.hpp"

namespace IMC
{
  //! TREX Operation.
  class TrexOperation: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Post Token.
      OP_POST_TOKEN = 1,
      //! Post Goal.
      OP_POST_GOAL = 2,
      //! Recall Goal.
      OP_RECALL_GOAL = 3,
      //! Request current plan.
      OP_REQUEST_PLAN = 4,
      //! Report current plan.
      OP_REPORT_PLAN = 5
    };

    //! Operation.
    uint8_t op;
    //! Goal Id.
    std::string goal_id;
    //! Token.
    InlineMessage<TrexToken> token;

    static uint16_t
    getIdStatic(void)
    {
      return 655;
    }

    static TrexOperation*
    cast(Message* msg__)
    {
      return (TrexOperation*)msg__;
    }

    TrexOperation(void)
    {
      m_header.mgid = TrexOperation::getIdStatic();
      clear();
      token.setParent(this);
    }

    TrexOperation*
    clone(void) const
    {
      return new TrexOperation(*this);
    }

    void
    clear(void)
    {
      op = 0;
      goal_id.clear();
      token.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TrexOperation& other__ = static_cast<const TrexOperation&>(msg__);
      if (op != other__.op) return false;
      if (goal_id != other__.goal_id) return false;
      if (token != other__.token) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(goal_id, ptr__);
      ptr__ += token.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(goal_id, bfr__, size__);
      bfr__ += token.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(goal_id, bfr__, size__);
      bfr__ += token.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexOperation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexOperation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(goal_id) + token.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "goal_id", goal_id, nindent__);
      token.toJSON(os__, "token", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!token.isNull())
      {
        token.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!token.isNull())
      {
        token.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!token.isNull())
      {
        token.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!token.isNull())
      {
        token.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!token.isNull())
      {
        token.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
