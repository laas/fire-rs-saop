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

#ifndef IMC_PLANSTATISTICS_HPP_INCLUDED_
#define IMC_PLANSTATISTICS_HPP_INCLUDED_

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
  //! Plan Statistics.
  class PlanStatistics: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Before Plan.
      TP_PREPLAN = 0,
      //! During Plan.
      TP_INPLAN = 1,
      //! After Plan.
      TP_POSTPLAN = 2
    };

    //! Properties.
    enum PropertiesBits
    {
      //! Basic Plan.
      PRP_BASIC = 0x00,
      //! Nonlinear.
      PRP_NONLINEAR = 0x01,
      //! Infinite.
      PRP_INFINITE = 0x02,
      //! Cyclical.
      PRP_CYCLICAL = 0x04,
      //! All.
      PRP_ALL = 0x07
    };

    //! Plan Identifier.
    std::string plan_id;
    //! Type.
    uint8_t type;
    //! Properties.
    uint8_t properties;
    //! Durations.
    std::string durations;
    //! Distances.
    std::string distances;
    //! Actions.
    std::string actions;
    //! Fuel.
    std::string fuel;

    static uint16_t
    getIdStatic(void)
    {
      return 564;
    }

    static PlanStatistics*
    cast(Message* msg__)
    {
      return (PlanStatistics*)msg__;
    }

    PlanStatistics(void)
    {
      m_header.mgid = PlanStatistics::getIdStatic();
      clear();
    }

    PlanStatistics*
    clone(void) const
    {
      return new PlanStatistics(*this);
    }

    void
    clear(void)
    {
      plan_id.clear();
      type = 0;
      properties = 0;
      durations.clear();
      distances.clear();
      actions.clear();
      fuel.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PlanStatistics& other__ = static_cast<const PlanStatistics&>(msg__);
      if (plan_id != other__.plan_id) return false;
      if (type != other__.type) return false;
      if (properties != other__.properties) return false;
      if (durations != other__.durations) return false;
      if (distances != other__.distances) return false;
      if (actions != other__.actions) return false;
      if (fuel != other__.fuel) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(properties, ptr__);
      ptr__ += IMC::serialize(durations, ptr__);
      ptr__ += IMC::serialize(distances, ptr__);
      ptr__ += IMC::serialize(actions, ptr__);
      ptr__ += IMC::serialize(fuel, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(properties, bfr__, size__);
      bfr__ += IMC::deserialize(durations, bfr__, size__);
      bfr__ += IMC::deserialize(distances, bfr__, size__);
      bfr__ += IMC::deserialize(actions, bfr__, size__);
      bfr__ += IMC::deserialize(fuel, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(properties, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(durations, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(distances, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(actions, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(fuel, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PlanStatistics::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PlanStatistics";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(durations) + IMC::getSerializationSize(distances) + IMC::getSerializationSize(actions) + IMC::getSerializationSize(fuel);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "properties", properties, nindent__);
      IMC::toJSON(os__, "durations", durations, nindent__);
      IMC::toJSON(os__, "distances", distances, nindent__);
      IMC::toJSON(os__, "actions", actions, nindent__);
      IMC::toJSON(os__, "fuel", fuel, nindent__);
    }
  };
}

#endif
