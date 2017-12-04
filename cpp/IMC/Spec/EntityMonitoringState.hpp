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

#ifndef IMC_ENTITYMONITORINGSTATE_HPP_INCLUDED_
#define IMC_ENTITYMONITORINGSTATE_HPP_INCLUDED_

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
  //! Entity Monitoring State.
  class EntityMonitoringState: public Message
  {
  public:
    //! Entities monitored - Count.
    uint8_t mcount;
    //! Entities monitored - Names.
    std::string mnames;
    //! Entities with errors - Count.
    uint8_t ecount;
    //! Entities with errors - Names.
    std::string enames;
    //! Entities with critical errors - Count.
    uint8_t ccount;
    //! Entities with critical errors - Names.
    std::string cnames;
    //! Last Error -- Description.
    std::string last_error;
    //! Last Error -- Time.
    double last_error_time;

    static uint16_t
    getIdStatic(void)
    {
      return 503;
    }

    static EntityMonitoringState*
    cast(Message* msg__)
    {
      return (EntityMonitoringState*)msg__;
    }

    EntityMonitoringState(void)
    {
      m_header.mgid = EntityMonitoringState::getIdStatic();
      clear();
    }

    EntityMonitoringState*
    clone(void) const
    {
      return new EntityMonitoringState(*this);
    }

    void
    clear(void)
    {
      mcount = 0;
      mnames.clear();
      ecount = 0;
      enames.clear();
      ccount = 0;
      cnames.clear();
      last_error.clear();
      last_error_time = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EntityMonitoringState& other__ = static_cast<const EntityMonitoringState&>(msg__);
      if (mcount != other__.mcount) return false;
      if (mnames != other__.mnames) return false;
      if (ecount != other__.ecount) return false;
      if (enames != other__.enames) return false;
      if (ccount != other__.ccount) return false;
      if (cnames != other__.cnames) return false;
      if (last_error != other__.last_error) return false;
      if (last_error_time != other__.last_error_time) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(mcount, ptr__);
      ptr__ += IMC::serialize(mnames, ptr__);
      ptr__ += IMC::serialize(ecount, ptr__);
      ptr__ += IMC::serialize(enames, ptr__);
      ptr__ += IMC::serialize(ccount, ptr__);
      ptr__ += IMC::serialize(cnames, ptr__);
      ptr__ += IMC::serialize(last_error, ptr__);
      ptr__ += IMC::serialize(last_error_time, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(mcount, bfr__, size__);
      bfr__ += IMC::deserialize(mnames, bfr__, size__);
      bfr__ += IMC::deserialize(ecount, bfr__, size__);
      bfr__ += IMC::deserialize(enames, bfr__, size__);
      bfr__ += IMC::deserialize(ccount, bfr__, size__);
      bfr__ += IMC::deserialize(cnames, bfr__, size__);
      bfr__ += IMC::deserialize(last_error, bfr__, size__);
      bfr__ += IMC::deserialize(last_error_time, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(mcount, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(mnames, bfr__, size__);
      bfr__ += IMC::deserialize(ecount, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(enames, bfr__, size__);
      bfr__ += IMC::deserialize(ccount, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cnames, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(last_error, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(last_error_time, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EntityMonitoringState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EntityMonitoringState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 11;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(mnames) + IMC::getSerializationSize(enames) + IMC::getSerializationSize(cnames) + IMC::getSerializationSize(last_error);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "mcount", mcount, nindent__);
      IMC::toJSON(os__, "mnames", mnames, nindent__);
      IMC::toJSON(os__, "ecount", ecount, nindent__);
      IMC::toJSON(os__, "enames", enames, nindent__);
      IMC::toJSON(os__, "ccount", ccount, nindent__);
      IMC::toJSON(os__, "cnames", cnames, nindent__);
      IMC::toJSON(os__, "last_error", last_error, nindent__);
      IMC::toJSON(os__, "last_error_time", last_error_time, nindent__);
    }
  };
}

#endif
