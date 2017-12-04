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

#ifndef IMC_PLANTRANSITION_HPP_INCLUDED_
#define IMC_PLANTRANSITION_HPP_INCLUDED_

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
  //! Plan Transition.
  class PlanTransition: public Message
  {
  public:
    //! Source.
    std::string source_man;
    //! Destination Maneuver Name.
    std::string dest_man;
    //! Transition conditions.
    std::string conditions;
    //! Transition actions.
    MessageList<Message> actions;

    static uint16_t
    getIdStatic(void)
    {
      return 553;
    }

    static PlanTransition*
    cast(Message* msg__)
    {
      return (PlanTransition*)msg__;
    }

    PlanTransition(void)
    {
      m_header.mgid = PlanTransition::getIdStatic();
      clear();
      actions.setParent(this);
    }

    PlanTransition*
    clone(void) const
    {
      return new PlanTransition(*this);
    }

    void
    clear(void)
    {
      source_man.clear();
      dest_man.clear();
      conditions.clear();
      actions.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanTransition& other__ = static_cast<const PlanTransition&>(msg__);
      if (source_man != other__.source_man) return false;
      if (dest_man != other__.dest_man) return false;
      if (conditions != other__.conditions) return false;
      if (actions != other__.actions) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(source_man, ptr__);
      ptr__ += IMC::serialize(dest_man, ptr__);
      ptr__ += IMC::serialize(conditions, ptr__);
      ptr__ += actions.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(source_man, bfr__, size__);
      bfr__ += IMC::deserialize(dest_man, bfr__, size__);
      bfr__ += IMC::deserialize(conditions, bfr__, size__);
      bfr__ += actions.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(source_man, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dest_man, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(conditions, bfr__, size__);
      bfr__ += actions.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanTransition::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanTransition";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(source_man) + IMC::getSerializationSize(dest_man) + IMC::getSerializationSize(conditions) + actions.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "source_man", source_man, nindent__);
      IMC::toJSON(os__, "dest_man", dest_man, nindent__);
      IMC::toJSON(os__, "conditions", conditions, nindent__);
      actions.toJSON(os__, "actions", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      actions.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      actions.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      actions.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      actions.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      actions.setDestinationEntity(value__);
    }
  };
}

#endif
