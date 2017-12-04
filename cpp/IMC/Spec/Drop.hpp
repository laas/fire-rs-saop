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

#ifndef IMC_DROP_HPP_INCLUDED_
#define IMC_DROP_HPP_INCLUDED_

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
  //! Drop Maneuver.
  class Drop: public Maneuver
  {
  public:
    //! Timeout.
    uint16_t timeout;
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Z Reference.
    float z;
    //! Z Units.
    uint8_t z_units;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 486;
    }

    static Drop*
    cast(Message* msg__)
    {
      return (Drop*)msg__;
    }

    Drop(void)
    {
      m_header.mgid = Drop::getIdStatic();
      clear();
    }

    Drop*
    clone(void) const
    {
      return new Drop(*this);
    }

    void
    clear(void)
    {
      timeout = 0;
      lat = 0;
      lon = 0;
      z = 0;
      z_units = 0;
      speed = 0;
      speed_units = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Drop& other__ = static_cast<const Drop&>(msg__);
      if (timeout != other__.timeout) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (z != other__.z) return false;
      if (z_units != other__.z_units) return false;
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
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
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
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Drop::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Drop";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 28;
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
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }
  };
}

#endif
