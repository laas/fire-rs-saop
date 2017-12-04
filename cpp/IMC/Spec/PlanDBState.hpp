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

#ifndef IMC_PLANDBSTATE_HPP_INCLUDED_
#define IMC_PLANDBSTATE_HPP_INCLUDED_

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
#include "../Spec/PlanDBInformation.hpp"

namespace IMC
{
  //! Plan DB State.
  class PlanDBState: public Message
  {
  public:
    //! Plan -- Count.
    uint16_t plan_count;
    //! Plan -- Size of all plans.
    uint32_t plan_size;
    //! Last Change -- Time.
    double change_time;
    //! Last Change -- Source Address.
    uint16_t change_sid;
    //! Last Change -- Source Name.
    std::string change_sname;
    //! MD5.
    std::vector<char> md5;
    //! Plan info.
    MessageList<PlanDBInformation> plans_info;

    static uint16_t
    getIdStatic(void)
    {
      return 557;
    }

    static PlanDBState*
    cast(Message* msg__)
    {
      return (PlanDBState*)msg__;
    }

    PlanDBState(void)
    {
      m_header.mgid = PlanDBState::getIdStatic();
      clear();
      plans_info.setParent(this);
    }

    PlanDBState*
    clone(void) const
    {
      return new PlanDBState(*this);
    }

    void
    clear(void)
    {
      plan_count = 0;
      plan_size = 0;
      change_time = 0;
      change_sid = 0;
      change_sname.clear();
      md5.clear();
      plans_info.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanDBState& other__ = static_cast<const PlanDBState&>(msg__);
      if (plan_count != other__.plan_count) return false;
      if (plan_size != other__.plan_size) return false;
      if (change_time != other__.change_time) return false;
      if (change_sid != other__.change_sid) return false;
      if (change_sname != other__.change_sname) return false;
      if (md5 != other__.md5) return false;
      if (plans_info != other__.plans_info) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(plan_count, ptr__);
      ptr__ += IMC::serialize(plan_size, ptr__);
      ptr__ += IMC::serialize(change_time, ptr__);
      ptr__ += IMC::serialize(change_sid, ptr__);
      ptr__ += IMC::serialize(change_sname, ptr__);
      ptr__ += IMC::serialize(md5, ptr__);
      ptr__ += plans_info.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(plan_count, bfr__, size__);
      bfr__ += IMC::deserialize(plan_size, bfr__, size__);
      bfr__ += IMC::deserialize(change_time, bfr__, size__);
      bfr__ += IMC::deserialize(change_sid, bfr__, size__);
      bfr__ += IMC::deserialize(change_sname, bfr__, size__);
      bfr__ += IMC::deserialize(md5, bfr__, size__);
      bfr__ += plans_info.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(plan_count, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_size, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_sid, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_sname, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(md5, bfr__, size__);
      bfr__ += plans_info.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanDBState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanDBState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 16;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(change_sname) + IMC::getSerializationSize(md5) + plans_info.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "plan_count", plan_count, nindent__);
      IMC::toJSON(os__, "plan_size", plan_size, nindent__);
      IMC::toJSON(os__, "change_time", change_time, nindent__);
      IMC::toJSON(os__, "change_sid", change_sid, nindent__);
      IMC::toJSON(os__, "change_sname", change_sname, nindent__);
      IMC::toJSON(os__, "md5", md5, nindent__);
      plans_info.toJSON(os__, "plans_info", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      plans_info.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      plans_info.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      plans_info.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      plans_info.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      plans_info.setDestinationEntity(value__);
    }
  };
}

#endif
