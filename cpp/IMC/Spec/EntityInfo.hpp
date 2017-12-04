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

#ifndef IMC_ENTITYINFO_HPP_INCLUDED_
#define IMC_ENTITYINFO_HPP_INCLUDED_

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
  //! Entity Information.
  class EntityInfo: public Message
  {
  public:
    //! Entity Identifier.
    uint8_t id;
    //! Label.
    std::string label;
    //! Component name.
    std::string component;
    //! Activation Time.
    uint16_t act_time;
    //! Deactivation Time.
    uint16_t deact_time;

    static uint16_t
    getIdStatic(void)
    {
      return 3;
    }

    static EntityInfo*
    cast(Message* msg__)
    {
      return (EntityInfo*)msg__;
    }

    EntityInfo(void)
    {
      m_header.mgid = EntityInfo::getIdStatic();
      clear();
    }

    EntityInfo*
    clone(void) const
    {
      return new EntityInfo(*this);
    }

    void
    clear(void)
    {
      id = 0;
      label.clear();
      component.clear();
      act_time = 0;
      deact_time = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EntityInfo& other__ = static_cast<const EntityInfo&>(msg__);
      if (id != other__.id) return false;
      if (label != other__.label) return false;
      if (component != other__.component) return false;
      if (act_time != other__.act_time) return false;
      if (deact_time != other__.deact_time) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(label, ptr__);
      ptr__ += IMC::serialize(component, ptr__);
      ptr__ += IMC::serialize(act_time, ptr__);
      ptr__ += IMC::serialize(deact_time, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(label, bfr__, size__);
      bfr__ += IMC::deserialize(component, bfr__, size__);
      bfr__ += IMC::deserialize(act_time, bfr__, size__);
      bfr__ += IMC::deserialize(deact_time, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(label, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(component, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(act_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(deact_time, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EntityInfo::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EntityInfo";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(label) + IMC::getSerializationSize(component);
    }

    uint16_t
    getSubId(void) const
    {
      return id;
    }

    void
    setSubId(uint16_t subid)
    {
      id = (uint8_t)subid;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      IMC::toJSON(os__, "label", label, nindent__);
      IMC::toJSON(os__, "component", component, nindent__);
      IMC::toJSON(os__, "act_time", act_time, nindent__);
      IMC::toJSON(os__, "deact_time", deact_time, nindent__);
    }
  };
}

#endif
