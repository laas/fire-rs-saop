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

#ifndef IMC_PLANDBINFORMATION_HPP_INCLUDED_
#define IMC_PLANDBINFORMATION_HPP_INCLUDED_

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
  //! Plan DB Information.
  class PlanDBInformation: public Message
  {
  public:
    //! Plan ID.
    std::string plan_id;
    //! Plan Size.
    uint16_t plan_size;
    //! Last Changed -- Time.
    double change_time;
    //! Last Change -- Source Address.
    uint16_t change_sid;
    //! Last Change -- Source Name.
    std::string change_sname;
    //! MD5.
    std::vector<char> md5;

    static uint16_t
    getIdStatic(void)
    {
      return 558;
    }

    static PlanDBInformation*
    cast(Message* msg__)
    {
      return (PlanDBInformation*)msg__;
    }

    PlanDBInformation(void)
    {
      m_header.mgid = PlanDBInformation::getIdStatic();
      clear();
    }

    PlanDBInformation*
    clone(void) const
    {
      return new PlanDBInformation(*this);
    }

    void
    clear(void)
    {
      plan_id.clear();
      plan_size = 0;
      change_time = 0;
      change_sid = 0;
      change_sname.clear();
      md5.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanDBInformation& other__ = static_cast<const PlanDBInformation&>(msg__);
      if (plan_id != other__.plan_id) return false;
      if (plan_size != other__.plan_size) return false;
      if (change_time != other__.change_time) return false;
      if (change_sid != other__.change_sid) return false;
      if (change_sname != other__.change_sname) return false;
      if (md5 != other__.md5) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(plan_size, ptr__);
      ptr__ += IMC::serialize(change_time, ptr__);
      ptr__ += IMC::serialize(change_sid, ptr__);
      ptr__ += IMC::serialize(change_sname, ptr__);
      ptr__ += IMC::serialize(md5, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(plan_size, bfr__, size__);
      bfr__ += IMC::deserialize(change_time, bfr__, size__);
      bfr__ += IMC::deserialize(change_sid, bfr__, size__);
      bfr__ += IMC::deserialize(change_sname, bfr__, size__);
      bfr__ += IMC::deserialize(md5, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_size, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_sid, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(change_sname, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(md5, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanDBInformation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanDBInformation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(change_sname) + IMC::getSerializationSize(md5);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "plan_size", plan_size, nindent__);
      IMC::toJSON(os__, "change_time", change_time, nindent__);
      IMC::toJSON(os__, "change_sid", change_sid, nindent__);
      IMC::toJSON(os__, "change_sname", change_sname, nindent__);
      IMC::toJSON(os__, "md5", md5, nindent__);
    }
  };
}

#endif
