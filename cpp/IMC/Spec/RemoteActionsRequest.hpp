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

#ifndef IMC_REMOTEACTIONSREQUEST_HPP_INCLUDED_
#define IMC_REMOTEACTIONSREQUEST_HPP_INCLUDED_

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
  //! Remote Actions Request.
  class RemoteActionsRequest: public Message
  {
  public:
    //! operation.
    enum operationEnum
    {
      //! Report.
      OP_REPORT = 0,
      //! Query.
      OP_QUERY = 1
    };

    //! operation.
    uint8_t op;
    //! Actions.
    std::string actions;

    static uint16_t
    getIdStatic(void)
    {
      return 304;
    }

    static RemoteActionsRequest*
    cast(Message* msg__)
    {
      return (RemoteActionsRequest*)msg__;
    }

    RemoteActionsRequest(void)
    {
      m_header.mgid = RemoteActionsRequest::getIdStatic();
      clear();
    }

    RemoteActionsRequest*
    clone(void) const
    {
      return new RemoteActionsRequest(*this);
    }

    void
    clear(void)
    {
      op = 0;
      actions.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RemoteActionsRequest& other__ = static_cast<const RemoteActionsRequest&>(msg__);
      if (op != other__.op) return false;
      if (actions != other__.actions) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(actions, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(actions, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(actions, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RemoteActionsRequest::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RemoteActionsRequest";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(actions);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "actions", actions, nindent__);
    }
  };
}

#endif
