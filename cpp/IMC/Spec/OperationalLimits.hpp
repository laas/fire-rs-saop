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

#ifndef IMC_OPERATIONALLIMITS_HPP_INCLUDED_
#define IMC_OPERATIONALLIMITS_HPP_INCLUDED_

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
  //! Operational Limits.
  class OperationalLimits: public Message
  {
  public:
    //! Field Indicator Mask.
    uint8_t mask;
    //! Maximum Depth.
    float max_depth;
    //! Minimum Altitude.
    float min_altitude;
    //! Maximum Altitude.
    float max_altitude;
    //! Minimum Speed.
    float min_speed;
    //! Maximum Speed.
    float max_speed;
    //! Maximum Vertical Rate.
    float max_vrate;
    //! Area -- WGS-84 Latitude.
    double lat;
    //! Area -- WGS-84 Longitude.
    double lon;
    //! Area -- Orientation.
    float orientation;
    //! Area -- Width.
    float width;
    //! Area -- Length.
    float length;

    static uint16_t
    getIdStatic(void)
    {
      return 504;
    }

    static OperationalLimits*
    cast(Message* msg__)
    {
      return (OperationalLimits*)msg__;
    }

    OperationalLimits(void)
    {
      m_header.mgid = OperationalLimits::getIdStatic();
      clear();
    }

    OperationalLimits*
    clone(void) const
    {
      return new OperationalLimits(*this);
    }

    void
    clear(void)
    {
      mask = 0;
      max_depth = 0;
      min_altitude = 0;
      max_altitude = 0;
      min_speed = 0;
      max_speed = 0;
      max_vrate = 0;
      lat = 0;
      lon = 0;
      orientation = 0;
      width = 0;
      length = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::OperationalLimits& other__ = static_cast<const OperationalLimits&>(msg__);
      if (mask != other__.mask) return false;
      if (max_depth != other__.max_depth) return false;
      if (min_altitude != other__.min_altitude) return false;
      if (max_altitude != other__.max_altitude) return false;
      if (min_speed != other__.min_speed) return false;
      if (max_speed != other__.max_speed) return false;
      if (max_vrate != other__.max_vrate) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (orientation != other__.orientation) return false;
      if (width != other__.width) return false;
      if (length != other__.length) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(mask, ptr__);
      ptr__ += IMC::serialize(max_depth, ptr__);
      ptr__ += IMC::serialize(min_altitude, ptr__);
      ptr__ += IMC::serialize(max_altitude, ptr__);
      ptr__ += IMC::serialize(min_speed, ptr__);
      ptr__ += IMC::serialize(max_speed, ptr__);
      ptr__ += IMC::serialize(max_vrate, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(orientation, ptr__);
      ptr__ += IMC::serialize(width, ptr__);
      ptr__ += IMC::serialize(length, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(mask, bfr__, size__);
      bfr__ += IMC::deserialize(max_depth, bfr__, size__);
      bfr__ += IMC::deserialize(min_altitude, bfr__, size__);
      bfr__ += IMC::deserialize(max_altitude, bfr__, size__);
      bfr__ += IMC::deserialize(min_speed, bfr__, size__);
      bfr__ += IMC::deserialize(max_speed, bfr__, size__);
      bfr__ += IMC::deserialize(max_vrate, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(orientation, bfr__, size__);
      bfr__ += IMC::deserialize(width, bfr__, size__);
      bfr__ += IMC::deserialize(length, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(mask, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(min_altitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_altitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(min_speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_vrate, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(orientation, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(width, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(length, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return OperationalLimits::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "OperationalLimits";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 53;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "mask", mask, nindent__);
      IMC::toJSON(os__, "max_depth", max_depth, nindent__);
      IMC::toJSON(os__, "min_altitude", min_altitude, nindent__);
      IMC::toJSON(os__, "max_altitude", max_altitude, nindent__);
      IMC::toJSON(os__, "min_speed", min_speed, nindent__);
      IMC::toJSON(os__, "max_speed", max_speed, nindent__);
      IMC::toJSON(os__, "max_vrate", max_vrate, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "orientation", orientation, nindent__);
      IMC::toJSON(os__, "width", width, nindent__);
      IMC::toJSON(os__, "length", length, nindent__);
    }
  };
}

#endif
