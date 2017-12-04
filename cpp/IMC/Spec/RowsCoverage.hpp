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

#ifndef IMC_ROWSCOVERAGE_HPP_INCLUDED_
#define IMC_ROWSCOVERAGE_HPP_INCLUDED_

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
  //! Rows Coverage.
  class RowsCoverage: public Maneuver
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Square Curve.
      FLG_SQUARE_CURVE = 0x01,
      //! First Curve Right.
      FLG_CURVE_RIGHT = 0x02
    };

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
    //! Bearing.
    double bearing;
    //! Cross Angle.
    double cross_angle;
    //! Width.
    float width;
    //! Length.
    float length;
    //! Curve Offset.
    uint8_t coff;
    //! Angular Aperture.
    float angaperture;
    //! Range.
    uint16_t range;
    //! Overlap.
    uint8_t overlap;
    //! Flags.
    uint8_t flags;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 488;
    }

    static RowsCoverage*
    cast(Message* msg__)
    {
      return (RowsCoverage*)msg__;
    }

    RowsCoverage(void)
    {
      m_header.mgid = RowsCoverage::getIdStatic();
      clear();
    }

    RowsCoverage*
    clone(void) const
    {
      return new RowsCoverage(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      z = 0;
      z_units = 0;
      speed = 0;
      speed_units = 0;
      bearing = 0;
      cross_angle = 0;
      width = 0;
      length = 0;
      coff = 0;
      angaperture = 0;
      range = 0;
      overlap = 0;
      flags = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RowsCoverage& other__ = static_cast<const RowsCoverage&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (z != other__.z) return false;
      if (z_units != other__.z_units) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (bearing != other__.bearing) return false;
      if (cross_angle != other__.cross_angle) return false;
      if (width != other__.width) return false;
      if (length != other__.length) return false;
      if (coff != other__.coff) return false;
      if (angaperture != other__.angaperture) return false;
      if (range != other__.range) return false;
      if (overlap != other__.overlap) return false;
      if (flags != other__.flags) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += IMC::serialize(bearing, ptr__);
      ptr__ += IMC::serialize(cross_angle, ptr__);
      ptr__ += IMC::serialize(width, ptr__);
      ptr__ += IMC::serialize(length, ptr__);
      ptr__ += IMC::serialize(coff, ptr__);
      ptr__ += IMC::serialize(angaperture, ptr__);
      ptr__ += IMC::serialize(range, ptr__);
      ptr__ += IMC::serialize(overlap, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(bearing, bfr__, size__);
      bfr__ += IMC::deserialize(cross_angle, bfr__, size__);
      bfr__ += IMC::deserialize(width, bfr__, size__);
      bfr__ += IMC::deserialize(length, bfr__, size__);
      bfr__ += IMC::deserialize(coff, bfr__, size__);
      bfr__ += IMC::deserialize(angaperture, bfr__, size__);
      bfr__ += IMC::deserialize(range, bfr__, size__);
      bfr__ += IMC::deserialize(overlap, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bearing, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cross_angle, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(width, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(length, bfr__, size__);
      bfr__ += IMC::deserialize(coff, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(angaperture, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(range, bfr__, size__);
      bfr__ += IMC::deserialize(overlap, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RowsCoverage::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RowsCoverage";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 59;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "bearing", bearing, nindent__);
      IMC::toJSON(os__, "cross_angle", cross_angle, nindent__);
      IMC::toJSON(os__, "width", width, nindent__);
      IMC::toJSON(os__, "length", length, nindent__);
      IMC::toJSON(os__, "coff", coff, nindent__);
      IMC::toJSON(os__, "angaperture", angaperture, nindent__);
      IMC::toJSON(os__, "range", range, nindent__);
      IMC::toJSON(os__, "overlap", overlap, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }
  };
}

#endif
