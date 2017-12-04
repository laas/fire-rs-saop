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

#ifndef IMC_PLANCONTROLSTATE_HPP_INCLUDED_
#define IMC_PLANCONTROLSTATE_HPP_INCLUDED_

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
  //! Plan Control State.
  class PlanControlState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Blocked.
      PCS_BLOCKED = 0,
      //! Ready.
      PCS_READY = 1,
      //! Initializing.
      PCS_INITIALIZING = 2,
      //! Executing.
      PCS_EXECUTING = 3
    };

    //! Last Plan Outcome.
    enum LastPlanOutcomeEnum
    {
      //! None.
      LPO_NONE = 0,
      //! Success.
      LPO_SUCCESS = 1,
      //! Failure.
      LPO_FAILURE = 2
    };

    //! State.
    uint8_t state;
    //! Plan -- ID.
    std::string plan_id;
    //! Plan -- ETA.
    int32_t plan_eta;
    //! Plan -- Progress.
    float plan_progress;
    //! Maneuver -- ID.
    std::string man_id;
    //! Maneuver -- Type.
    uint16_t man_type;
    //! Maneuver -- ETA.
    int32_t man_eta;
    //! Last Plan Outcome.
    uint8_t last_outcome;

    static uint16_t
    getIdStatic(void)
    {
      return 560;
    }

    static PlanControlState*
    cast(Message* msg__)
    {
      return (PlanControlState*)msg__;
    }

    PlanControlState(void)
    {
      m_header.mgid = PlanControlState::getIdStatic();
      clear();
    }

    PlanControlState*
    clone(void) const
    {
      return new PlanControlState(*this);
    }

    void
    clear(void)
    {
      state = 0;
      plan_id.clear();
      plan_eta = 0;
      plan_progress = 0;
      man_id.clear();
      man_type = 0;
      man_eta = 0;
      last_outcome = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanControlState& other__ = static_cast<const PlanControlState&>(msg__);
      if (state != other__.state) return false;
      if (plan_id != other__.plan_id) return false;
      if (plan_eta != other__.plan_eta) return false;
      if (plan_progress != other__.plan_progress) return false;
      if (man_id != other__.man_id) return false;
      if (man_type != other__.man_type) return false;
      if (man_eta != other__.man_eta) return false;
      if (last_outcome != other__.last_outcome) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(plan_eta, ptr__);
      ptr__ += IMC::serialize(plan_progress, ptr__);
      ptr__ += IMC::serialize(man_id, ptr__);
      ptr__ += IMC::serialize(man_type, ptr__);
      ptr__ += IMC::serialize(man_eta, ptr__);
      ptr__ += IMC::serialize(last_outcome, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(plan_eta, bfr__, size__);
      bfr__ += IMC::deserialize(plan_progress, bfr__, size__);
      bfr__ += IMC::deserialize(man_id, bfr__, size__);
      bfr__ += IMC::deserialize(man_type, bfr__, size__);
      bfr__ += IMC::deserialize(man_eta, bfr__, size__);
      bfr__ += IMC::deserialize(last_outcome, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_eta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_progress, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(man_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(man_type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(man_eta, bfr__, size__);
      bfr__ += IMC::deserialize(last_outcome, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanControlState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanControlState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 16;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(man_id);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "plan_eta", plan_eta, nindent__);
      IMC::toJSON(os__, "plan_progress", plan_progress, nindent__);
      IMC::toJSON(os__, "man_id", man_id, nindent__);
      IMC::toJSON(os__, "man_type", man_type, nindent__);
      IMC::toJSON(os__, "man_eta", man_eta, nindent__);
      IMC::toJSON(os__, "last_outcome", last_outcome, nindent__);
    }
  };
}

#endif
