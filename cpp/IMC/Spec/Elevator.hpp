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

#ifndef IMC_ELEVATOR_HPP_INCLUDED_
#define IMC_ELEVATOR_HPP_INCLUDED_

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
  //! Elevator Maneuver.
  class Elevator: public Maneuver
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Start from current position.
      FLG_CURR_POS = 0x01
    };

    //! Timeout.
    uint16_t timeout;
    //! Flags.
    uint8_t flags;
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Start Point -- Z Reference.
    float start_z;
    //! Start Point -- Z Units.
    uint8_t start_z_units;
    //! End Point -- Z Reference.
    float end_z;
    //! End Point -- Z Units.
    uint8_t end_z_units;
    //! Radius.
    float radius;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 462;
    }

    static Elevator*
    cast(Message* msg__)
    {
      return (Elevator*)msg__;
    }

    Elevator(void)
    {
      m_header.mgid = Elevator::getIdStatic();
      clear();
    }

    Elevator*
    clone(void) const
    {
      return new Elevator(*this);
    }

    void
    clear(void)
    {
      timeout = 0;
      flags = 0;
      lat = 0;
      lon = 0;
      start_z = 0;
      start_z_units = 0;
      end_z = 0;
      end_z_units = 0;
      radius = 0;
      speed = 0;
      speed_units = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Elevator& other__ = static_cast<const Elevator&>(msg__);
      if (timeout != other__.timeout) return false;
      if (flags != other__.flags) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (start_z != other__.start_z) return false;
      if (start_z_units != other__.start_z_units) return false;
      if (end_z != other__.end_z) return false;
      if (end_z_units != other__.end_z_units) return false;
      if (radius != other__.radius) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(start_z, ptr__);
      ptr__ += IMC::serialize(start_z_units, ptr__);
      ptr__ += IMC::serialize(end_z, ptr__);
      ptr__ += IMC::serialize(end_z_units, ptr__);
      ptr__ += IMC::serialize(radius, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(start_z, bfr__, size__);
      bfr__ += IMC::deserialize(start_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(end_z, bfr__, size__);
      bfr__ += IMC::deserialize(end_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(radius, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_z, bfr__, size__);
      bfr__ += IMC::deserialize(start_z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(end_z, bfr__, size__);
      bfr__ += IMC::deserialize(end_z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(radius, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Elevator::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Elevator";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 38;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "start_z", start_z, nindent__);
      IMC::toJSON(os__, "start_z_units", start_z_units, nindent__);
      IMC::toJSON(os__, "end_z", end_z, nindent__);
      IMC::toJSON(os__, "end_z_units", end_z_units, nindent__);
      IMC::toJSON(os__, "radius", radius, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }
  };
}

#endif
