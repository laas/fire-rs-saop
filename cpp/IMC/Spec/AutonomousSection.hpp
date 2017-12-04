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

#ifndef IMC_AUTONOMOUSSECTION_HPP_INCLUDED_
#define IMC_AUTONOMOUSSECTION_HPP_INCLUDED_

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
#include "../Spec/PolygonVertex.hpp"
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Autonomous Section.
  class AutonomousSection: public Maneuver
  {
  public:
    //! Enforced Limits.
    enum EnforcedLimitsBits
    {
      //! Maximum Depth Limit.
      ENFORCE_DEPTH = 0x01,
      //! Minimum Altitude Limit.
      ENFORCE_ALTITUDE = 0x02,
      //! Time Limit.
      ENFORCE_TIMEOUT = 0x04,
      //! Polygonal Area Limits.
      ENFORCE_AREA2D = 0x08
    };

    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Enforced Limits.
    uint8_t limits;
    //! Maximum depth.
    double max_depth;
    //! Minimum altitude.
    double min_alt;
    //! Time Limit.
    double time_limit;
    //! Area Limits.
    MessageList<PolygonVertex> area_limits;
    //! Controller.
    std::string controller;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 493;
    }

    static AutonomousSection*
    cast(Message* msg__)
    {
      return (AutonomousSection*)msg__;
    }

    AutonomousSection(void)
    {
      m_header.mgid = AutonomousSection::getIdStatic();
      clear();
      area_limits.setParent(this);
    }

    AutonomousSection*
    clone(void) const
    {
      return new AutonomousSection(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      speed = 0;
      speed_units = 0;
      limits = 0;
      max_depth = 0;
      min_alt = 0;
      time_limit = 0;
      area_limits.clear();
      controller.clear();
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AutonomousSection& other__ = static_cast<const AutonomousSection&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (limits != other__.limits) return false;
      if (max_depth != other__.max_depth) return false;
      if (min_alt != other__.min_alt) return false;
      if (time_limit != other__.time_limit) return false;
      if (area_limits != other__.area_limits) return false;
      if (controller != other__.controller) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += IMC::serialize(limits, ptr__);
      ptr__ += IMC::serialize(max_depth, ptr__);
      ptr__ += IMC::serialize(min_alt, ptr__);
      ptr__ += IMC::serialize(time_limit, ptr__);
      ptr__ += area_limits.serialize(ptr__);
      ptr__ += IMC::serialize(controller, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(limits, bfr__, size__);
      bfr__ += IMC::deserialize(max_depth, bfr__, size__);
      bfr__ += IMC::deserialize(min_alt, bfr__, size__);
      bfr__ += IMC::deserialize(time_limit, bfr__, size__);
      bfr__ += area_limits.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(controller, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(limits, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(min_alt, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_limit, bfr__, size__);
      bfr__ += area_limits.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(controller, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AutonomousSection::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AutonomousSection";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 46;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return area_limits.getSerializationSize() + IMC::getSerializationSize(controller) + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "limits", limits, nindent__);
      IMC::toJSON(os__, "max_depth", max_depth, nindent__);
      IMC::toJSON(os__, "min_alt", min_alt, nindent__);
      IMC::toJSON(os__, "time_limit", time_limit, nindent__);
      area_limits.toJSON(os__, "area_limits", nindent__);
      IMC::toJSON(os__, "controller", controller, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      area_limits.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      area_limits.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      area_limits.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      area_limits.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      area_limits.setDestinationEntity(value__);
    }
  };
}

#endif
