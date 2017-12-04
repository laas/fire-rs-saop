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

#ifndef IMC_EMERGENCYCONTROLSTATE_HPP_INCLUDED_
#define IMC_EMERGENCYCONTROLSTATE_HPP_INCLUDED_

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
  //! Emergency Control State.
  class EmergencyControlState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Not Configured.
      ECS_NOT_CONFIGURED = 0,
      //! Disabled.
      ECS_DISABLED = 1,
      //! Enabled.
      ECS_ENABLED = 2,
      //! Armed.
      ECS_ARMED = 3,
      //! Active.
      ECS_ACTIVE = 4,
      //! Stopping.
      ECS_STOPPING = 5
    };

    //! State.
    uint8_t state;
    //! Plan Id.
    std::string plan_id;
    //! Communications Level.
    uint8_t comm_level;

    static uint16_t
    getIdStatic(void)
    {
      return 555;
    }

    static EmergencyControlState*
    cast(Message* msg__)
    {
      return (EmergencyControlState*)msg__;
    }

    EmergencyControlState(void)
    {
      m_header.mgid = EmergencyControlState::getIdStatic();
      clear();
    }

    EmergencyControlState*
    clone(void) const
    {
      return new EmergencyControlState(*this);
    }

    void
    clear(void)
    {
      state = 0;
      plan_id.clear();
      comm_level = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EmergencyControlState& other__ = static_cast<const EmergencyControlState&>(msg__);
      if (state != other__.state) return false;
      if (plan_id != other__.plan_id) return false;
      if (comm_level != other__.comm_level) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(comm_level, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(comm_level, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(comm_level, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EmergencyControlState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EmergencyControlState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "comm_level", comm_level, nindent__);
    }
  };
}

#endif
