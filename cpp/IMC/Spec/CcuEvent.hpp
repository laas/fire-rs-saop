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

#ifndef IMC_CCUEVENT_HPP_INCLUDED_
#define IMC_CCUEVENT_HPP_INCLUDED_

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
  //! CCU Event.
  class CcuEvent: public Message
  {
  public:
    //! Event Type.
    enum EventTypeEnum
    {
      //! Log Book Entry Added.
      EVT_LOG_ENTRY = 1,
      //! Plan Added.
      EVT_PLAN_ADDED = 2,
      //! Plan Removed.
      EVT_PLAN_REMOVED = 3,
      //! Plan Changed.
      EVT_PLAN_CHANGED = 4,
      //! Map feature added.
      EVT_MAP_FEATURE_ADDED = 5,
      //! Map feature removed.
      EVT_MAP_FEATURE_REMOVED = 6,
      //! Map feature changed.
      EVT_MAP_FEATURE_CHANGED = 7,
      //! The sender is now teleoperating the vehicle.
      EVT_TELEOPERATION_STARTED = 8,
      //! The sender stopped teleoperating the vehicle.
      EVT_TELEOPERATION_ENDED = 9
    };

    //! Event Type.
    uint8_t type;
    //! Identifier.
    std::string id;
    //! Additional Data.
    InlineMessage<Message> arg;

    static uint16_t
    getIdStatic(void)
    {
      return 606;
    }

    static CcuEvent*
    cast(Message* msg__)
    {
      return (CcuEvent*)msg__;
    }

    CcuEvent(void)
    {
      m_header.mgid = CcuEvent::getIdStatic();
      clear();
      arg.setParent(this);
    }

    CcuEvent*
    clone(void) const
    {
      return new CcuEvent(*this);
    }

    void
    clear(void)
    {
      type = 0;
      id.clear();
      arg.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::CcuEvent& other__ = static_cast<const CcuEvent&>(msg__);
      if (type != other__.type) return false;
      if (id != other__.id) return false;
      if (arg != other__.arg) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += arg.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += arg.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(id, bfr__, size__);
      bfr__ += arg.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return CcuEvent::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "CcuEvent";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(id) + arg.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "id", id, nindent__);
      arg.toJSON(os__, "arg", nindent__);
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
