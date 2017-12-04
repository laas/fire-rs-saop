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

#ifndef IMC_PLANSPECIFICATION_HPP_INCLUDED_
#define IMC_PLANSPECIFICATION_HPP_INCLUDED_

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
#include "../Spec/PlanVariable.hpp"
#include "../Spec/PlanManeuver.hpp"
#include "../Spec/PlanTransition.hpp"

namespace IMC
{
  //! Plan Specification.
  class PlanSpecification: public Message
  {
  public:
    //! Plan ID.
    std::string plan_id;
    //! Plan Description.
    std::string description;
    //! Namespace.
    std::string vnamespace;
    //! Plan Variables.
    MessageList<PlanVariable> variables;
    //! Starting maneuver.
    std::string start_man_id;
    //! Maneuvers.
    MessageList<PlanManeuver> maneuvers;
    //! Transitions.
    MessageList<PlanTransition> transitions;
    //! Start Actions.
    MessageList<Message> start_actions;
    //! End Actions.
    MessageList<Message> end_actions;

    static uint16_t
    getIdStatic(void)
    {
      return 551;
    }

    static PlanSpecification*
    cast(Message* msg__)
    {
      return (PlanSpecification*)msg__;
    }

    PlanSpecification(void)
    {
      m_header.mgid = PlanSpecification::getIdStatic();
      clear();
      variables.setParent(this);
      maneuvers.setParent(this);
      transitions.setParent(this);
      start_actions.setParent(this);
      end_actions.setParent(this);
    }

    PlanSpecification*
    clone(void) const
    {
      return new PlanSpecification(*this);
    }

    void
    clear(void)
    {
      plan_id.clear();
      description.clear();
      vnamespace.clear();
      variables.clear();
      start_man_id.clear();
      maneuvers.clear();
      transitions.clear();
      start_actions.clear();
      end_actions.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanSpecification& other__ = static_cast<const PlanSpecification&>(msg__);
      if (plan_id != other__.plan_id) return false;
      if (description != other__.description) return false;
      if (vnamespace != other__.vnamespace) return false;
      if (variables != other__.variables) return false;
      if (start_man_id != other__.start_man_id) return false;
      if (maneuvers != other__.maneuvers) return false;
      if (transitions != other__.transitions) return false;
      if (start_actions != other__.start_actions) return false;
      if (end_actions != other__.end_actions) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(description, ptr__);
      ptr__ += IMC::serialize(vnamespace, ptr__);
      ptr__ += variables.serialize(ptr__);
      ptr__ += IMC::serialize(start_man_id, ptr__);
      ptr__ += maneuvers.serialize(ptr__);
      ptr__ += transitions.serialize(ptr__);
      ptr__ += start_actions.serialize(ptr__);
      ptr__ += end_actions.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(description, bfr__, size__);
      bfr__ += IMC::deserialize(vnamespace, bfr__, size__);
      bfr__ += variables.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(start_man_id, bfr__, size__);
      bfr__ += maneuvers.deserialize(bfr__, size__);
      bfr__ += transitions.deserialize(bfr__, size__);
      bfr__ += start_actions.deserialize(bfr__, size__);
      bfr__ += end_actions.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(description, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vnamespace, bfr__, size__);
      bfr__ += variables.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_man_id, bfr__, size__);
      bfr__ += maneuvers.reverseDeserialize(bfr__, size__);
      bfr__ += transitions.reverseDeserialize(bfr__, size__);
      bfr__ += start_actions.reverseDeserialize(bfr__, size__);
      bfr__ += end_actions.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanSpecification::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanSpecification";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(description) + IMC::getSerializationSize(vnamespace) + variables.getSerializationSize() + IMC::getSerializationSize(start_man_id) + maneuvers.getSerializationSize() + transitions.getSerializationSize() + start_actions.getSerializationSize() + end_actions.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "description", description, nindent__);
      IMC::toJSON(os__, "vnamespace", vnamespace, nindent__);
      variables.toJSON(os__, "variables", nindent__);
      IMC::toJSON(os__, "start_man_id", start_man_id, nindent__);
      maneuvers.toJSON(os__, "maneuvers", nindent__);
      transitions.toJSON(os__, "transitions", nindent__);
      start_actions.toJSON(os__, "start_actions", nindent__);
      end_actions.toJSON(os__, "end_actions", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      variables.setTimeStamp(value__);

      maneuvers.setTimeStamp(value__);

      transitions.setTimeStamp(value__);

      start_actions.setTimeStamp(value__);

      end_actions.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      variables.setSource(value__);

      maneuvers.setSource(value__);

      transitions.setSource(value__);

      start_actions.setSource(value__);

      end_actions.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      variables.setSourceEntity(value__);

      maneuvers.setSourceEntity(value__);

      transitions.setSourceEntity(value__);

      start_actions.setSourceEntity(value__);

      end_actions.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      variables.setDestination(value__);

      maneuvers.setDestination(value__);

      transitions.setDestination(value__);

      start_actions.setDestination(value__);

      end_actions.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      variables.setDestinationEntity(value__);

      maneuvers.setDestinationEntity(value__);

      transitions.setDestinationEntity(value__);

      start_actions.setDestinationEntity(value__);

      end_actions.setDestinationEntity(value__);
    }
  };
}

#endif
