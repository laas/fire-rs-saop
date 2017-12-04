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

#ifndef IMC_CACHECONTROL_HPP_INCLUDED_
#define IMC_CACHECONTROL_HPP_INCLUDED_

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
  //! Cache Control.
  class CacheControl: public Message
  {
  public:
    //! Control Operation.
    enum ControlOperationEnum
    {
      //! Store.
      COP_STORE = 0,
      //! Load.
      COP_LOAD = 1,
      //! Clear.
      COP_CLEAR = 2,
      //! Copy Snapshot.
      COP_COPY = 3,
      //! Snapshot Copy Complete.
      COP_COPY_COMPLETE = 4
    };

    //! Control Operation.
    uint8_t op;
    //! Snapshot destination.
    std::string snapshot;
    //! Message.
    InlineMessage<Message> message;

    static uint16_t
    getIdStatic(void)
    {
      return 101;
    }

    static CacheControl*
    cast(Message* msg__)
    {
      return (CacheControl*)msg__;
    }

    CacheControl(void)
    {
      m_header.mgid = CacheControl::getIdStatic();
      clear();
      message.setParent(this);
    }

    CacheControl*
    clone(void) const
    {
      return new CacheControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      snapshot.clear();
      message.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::CacheControl& other__ = static_cast<const CacheControl&>(msg__);
      if (op != other__.op) return false;
      if (snapshot != other__.snapshot) return false;
      if (message != other__.message) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(snapshot, ptr__);
      ptr__ += message.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(snapshot, bfr__, size__);
      bfr__ += message.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(snapshot, bfr__, size__);
      bfr__ += message.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return CacheControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "CacheControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(snapshot) + message.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "snapshot", snapshot, nindent__);
      message.toJSON(os__, "message", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!message.isNull())
      {
        message.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!message.isNull())
      {
        message.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!message.isNull())
      {
        message.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!message.isNull())
      {
        message.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!message.isNull())
      {
        message.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
