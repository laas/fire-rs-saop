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

#ifndef IMC_COMMSRELAY_HPP_INCLUDED_
#define IMC_COMMSRELAY_HPP_INCLUDED_

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
  //! Communications Relay.
  class CommsRelay: public Maneuver
  {
  public:
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Duration.
    uint16_t duration;
    //! System A.
    uint16_t sys_a;
    //! System B.
    uint16_t sys_b;
    //! Move threshold.
    float move_threshold;

    static uint16_t
    getIdStatic(void)
    {
      return 472;
    }

    static CommsRelay*
    cast(Message* msg__)
    {
      return (CommsRelay*)msg__;
    }

    CommsRelay(void)
    {
      m_header.mgid = CommsRelay::getIdStatic();
      clear();
    }

    CommsRelay*
    clone(void) const
    {
      return new CommsRelay(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      speed = 0;
      speed_units = 0;
      duration = 0;
      sys_a = 0;
      sys_b = 0;
      move_threshold = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::CommsRelay& other__ = static_cast<const CommsRelay&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (duration != other__.duration) return false;
      if (sys_a != other__.sys_a) return false;
      if (sys_b != other__.sys_b) return false;
      if (move_threshold != other__.move_threshold) return false;
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
      ptr__ += IMC::serialize(duration, ptr__);
      ptr__ += IMC::serialize(sys_a, ptr__);
      ptr__ += IMC::serialize(sys_b, ptr__);
      ptr__ += IMC::serialize(move_threshold, ptr__);
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
      bfr__ += IMC::deserialize(duration, bfr__, size__);
      bfr__ += IMC::deserialize(sys_a, bfr__, size__);
      bfr__ += IMC::deserialize(sys_b, bfr__, size__);
      bfr__ += IMC::deserialize(move_threshold, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(duration, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sys_a, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sys_b, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(move_threshold, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return CommsRelay::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "CommsRelay";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 31;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "duration", duration, nindent__);
      IMC::toJSON(os__, "sys_a", sys_a, nindent__);
      IMC::toJSON(os__, "sys_b", sys_b, nindent__);
      IMC::toJSON(os__, "move_threshold", move_threshold, nindent__);
    }
  };
}

#endif
