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

#ifndef IMC_PLANMANEUVER_HPP_INCLUDED_
#define IMC_PLANMANEUVER_HPP_INCLUDED_

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
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Plan Maneuver.
  class PlanManeuver: public Message
  {
  public:
    //! Maneuver ID.
    std::string maneuver_id;
    //! Maneuver Specification.
    InlineMessage<Maneuver> data;
    //! Start Actions.
    MessageList<Message> start_actions;
    //! End Actions.
    MessageList<Message> end_actions;

    static uint16_t
    getIdStatic(void)
    {
      return 552;
    }

    static PlanManeuver*
    cast(Message* msg__)
    {
      return (PlanManeuver*)msg__;
    }

    PlanManeuver(void)
    {
      m_header.mgid = PlanManeuver::getIdStatic();
      clear();
      data.setParent(this);
      start_actions.setParent(this);
      end_actions.setParent(this);
    }

    PlanManeuver*
    clone(void) const
    {
      return new PlanManeuver(*this);
    }

    void
    clear(void)
    {
      maneuver_id.clear();
      data.clear();
      start_actions.clear();
      end_actions.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanManeuver& other__ = static_cast<const PlanManeuver&>(msg__);
      if (maneuver_id != other__.maneuver_id) return false;
      if (data != other__.data) return false;
      if (start_actions != other__.start_actions) return false;
      if (end_actions != other__.end_actions) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(maneuver_id, ptr__);
      ptr__ += data.serialize(ptr__);
      ptr__ += start_actions.serialize(ptr__);
      ptr__ += end_actions.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(maneuver_id, bfr__, size__);
      bfr__ += data.deserialize(bfr__, size__);
      bfr__ += start_actions.deserialize(bfr__, size__);
      bfr__ += end_actions.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(maneuver_id, bfr__, size__);
      bfr__ += data.reverseDeserialize(bfr__, size__);
      bfr__ += start_actions.reverseDeserialize(bfr__, size__);
      bfr__ += end_actions.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanManeuver::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanManeuver";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(maneuver_id) + data.getSerializationSize() + start_actions.getSerializationSize() + end_actions.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "maneuver_id", maneuver_id, nindent__);
      data.toJSON(os__, "data", nindent__);
      start_actions.toJSON(os__, "start_actions", nindent__);
      end_actions.toJSON(os__, "end_actions", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!data.isNull())
      {
        data.get()->setTimeStamp(value__);
      }

      start_actions.setTimeStamp(value__);

      end_actions.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setSource(value__);
      }

      start_actions.setSource(value__);

      end_actions.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setSourceEntity(value__);
      }

      start_actions.setSourceEntity(value__);

      end_actions.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setDestination(value__);
      }

      start_actions.setDestination(value__);

      end_actions.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setDestinationEntity(value__);
      }

      start_actions.setDestinationEntity(value__);

      end_actions.setDestinationEntity(value__);
    }
  };
}

#endif
