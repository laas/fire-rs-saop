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

#ifndef IMC_SCHEDULEDGOTO_HPP_INCLUDED_
#define IMC_SCHEDULEDGOTO_HPP_INCLUDED_

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
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Scheduled Goto.
  class ScheduledGoto: public Maneuver
  {
  public:
    //! Delayed Behavior.
    enum DelayedBehaviorEnum
    {
      //! Resume.
      DBEH_RESUME = 0,
      //! Skip.
      DBEH_SKIP = 1,
      //! Fail.
      DBEH_FAIL = 2
    };

    //! Time of arrival.
    double arrival_time;
    //! Destination Latitude WGS-84.
    double lat;
    //! Destination Longitude WGS-84.
    double lon;
    //! Destination Z Reference.
    float z;
    //! Z Units.
    uint8_t z_units;
    //! Travel Z Reference.
    float travel_z;
    //! Travel Z Units.
    uint8_t travel_z_units;
    //! Delayed Behavior.
    uint8_t delayed;

    static uint16_t
    getIdStatic(void)
    {
      return 487;
    }

    static ScheduledGoto*
    cast(Message* msg__)
    {
      return (ScheduledGoto*)msg__;
    }

    ScheduledGoto(void)
    {
      m_header.mgid = ScheduledGoto::getIdStatic();
      clear();
    }

    ScheduledGoto*
    clone(void) const
    {
      return new ScheduledGoto(*this);
    }

    void
    clear(void)
    {
      arrival_time = 0;
      lat = 0;
      lon = 0;
      z = 0;
      z_units = 0;
      travel_z = 0;
      travel_z_units = 0;
      delayed = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ScheduledGoto& other__ = static_cast<const ScheduledGoto&>(msg__);
      if (arrival_time != other__.arrival_time) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (z != other__.z) return false;
      if (z_units != other__.z_units) return false;
      if (travel_z != other__.travel_z) return false;
      if (travel_z_units != other__.travel_z_units) return false;
      if (delayed != other__.delayed) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(arrival_time, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
      ptr__ += IMC::serialize(travel_z, ptr__);
      ptr__ += IMC::serialize(travel_z_units, ptr__);
      ptr__ += IMC::serialize(delayed, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(arrival_time, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::deserialize(travel_z, bfr__, size__);
      bfr__ += IMC::deserialize(travel_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(delayed, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(arrival_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(travel_z, bfr__, size__);
      bfr__ += IMC::deserialize(travel_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(delayed, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ScheduledGoto::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ScheduledGoto";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 35;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "arrival_time", arrival_time, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
      IMC::toJSON(os__, "travel_z", travel_z, nindent__);
      IMC::toJSON(os__, "travel_z_units", travel_z_units, nindent__);
      IMC::toJSON(os__, "delayed", delayed, nindent__);
    }
  };
}

#endif
