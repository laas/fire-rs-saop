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

#ifndef IMC_EMERGENCYCONTROL_HPP_INCLUDED_
#define IMC_EMERGENCYCONTROL_HPP_INCLUDED_

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
#include "../Spec/PlanSpecification.hpp"

namespace IMC
{
  //! Emergency Control.
  class EmergencyControl: public Message
  {
  public:
    //! Command.
    enum CommandEnum
    {
      //! Enable.
      ECTL_ENABLE = 0,
      //! Disable.
      ECTL_DISABLE = 1,
      //! Start.
      ECTL_START = 2,
      //! Stop.
      ECTL_STOP = 3,
      //! Query.
      ECTL_QUERY = 4,
      //! Set Plan.
      ECTL_SET_PLAN = 5
    };

    //! Command.
    uint8_t command;
    //! Plan Specification.
    InlineMessage<PlanSpecification> plan;

    static uint16_t
    getIdStatic(void)
    {
      return 554;
    }

    static EmergencyControl*
    cast(Message* msg__)
    {
      return (EmergencyControl*)msg__;
    }

    EmergencyControl(void)
    {
      m_header.mgid = EmergencyControl::getIdStatic();
      clear();
      plan.setParent(this);
    }

    EmergencyControl*
    clone(void) const
    {
      return new EmergencyControl(*this);
    }

    void
    clear(void)
    {
      command = 0;
      plan.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EmergencyControl& other__ = static_cast<const EmergencyControl&>(msg__);
      if (command != other__.command) return false;
      if (plan != other__.plan) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(command, ptr__);
      ptr__ += plan.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += plan.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += plan.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EmergencyControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EmergencyControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return plan.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "command", command, nindent__);
      plan.toJSON(os__, "plan", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!plan.isNull())
      {
        plan.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!plan.isNull())
      {
        plan.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!plan.isNull())
      {
        plan.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!plan.isNull())
      {
        plan.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!plan.isNull())
      {
        plan.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
