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

#ifndef IMC_DESIREDPATH_HPP_INCLUDED_
#define IMC_DESIREDPATH_HPP_INCLUDED_

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
#include "../Spec/ControlCommand.hpp"

namespace IMC
{
  //! Desired Path.
  class DesiredPath: public ControlCommand
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Start Point.
      FL_START = 0x01,
      //! Direct.
      FL_DIRECT = 0x02,
      //! No Altitude/Depth control.
      FL_NO_Z = 0x04,
      //! 3D Tracking.
      FL_3DTRACK = 0x08,
      //! Counter-Clockwise loiter.
      FL_CCLOCKW = 0x10,
      //! Loiter from current position.
      FL_LOITER_CURR = 0x20,
      //! Takeoff.
      FL_TAKEOFF = 0x40,
      //! Land.
      FL_LAND = 0x80
    };

    //! Path Reference.
    uint32_t path_ref;
    //! Start Point -- Latitude WGS-84.
    double start_lat;
    //! Start Point -- WGS-84 Longitude.
    double start_lon;
    //! Start Point -- Z Reference.
    float start_z;
    //! Start Point -- Z Units.
    uint8_t start_z_units;
    //! End Point -- WGS84 Latitude.
    double end_lat;
    //! End Point -- WGS-84 Longitude.
    double end_lon;
    //! End Point -- Z Reference.
    float end_z;
    //! End Point -- Z Units.
    uint8_t end_z_units;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Loiter -- Radius.
    float lradius;
    //! Flags.
    uint8_t flags;

    static uint16_t
    getIdStatic(void)
    {
      return 406;
    }

    static DesiredPath*
    cast(Message* msg__)
    {
      return (DesiredPath*)msg__;
    }

    DesiredPath(void)
    {
      m_header.mgid = DesiredPath::getIdStatic();
      clear();
    }

    DesiredPath*
    clone(void) const
    {
      return new DesiredPath(*this);
    }

    void
    clear(void)
    {
      path_ref = 0;
      start_lat = 0;
      start_lon = 0;
      start_z = 0;
      start_z_units = 0;
      end_lat = 0;
      end_lon = 0;
      end_z = 0;
      end_z_units = 0;
      speed = 0;
      speed_units = 0;
      lradius = 0;
      flags = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DesiredPath& other__ = static_cast<const DesiredPath&>(msg__);
      if (path_ref != other__.path_ref) return false;
      if (start_lat != other__.start_lat) return false;
      if (start_lon != other__.start_lon) return false;
      if (start_z != other__.start_z) return false;
      if (start_z_units != other__.start_z_units) return false;
      if (end_lat != other__.end_lat) return false;
      if (end_lon != other__.end_lon) return false;
      if (end_z != other__.end_z) return false;
      if (end_z_units != other__.end_z_units) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (lradius != other__.lradius) return false;
      if (flags != other__.flags) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(path_ref, ptr__);
      ptr__ += IMC::serialize(start_lat, ptr__);
      ptr__ += IMC::serialize(start_lon, ptr__);
      ptr__ += IMC::serialize(start_z, ptr__);
      ptr__ += IMC::serialize(start_z_units, ptr__);
      ptr__ += IMC::serialize(end_lat, ptr__);
      ptr__ += IMC::serialize(end_lon, ptr__);
      ptr__ += IMC::serialize(end_z, ptr__);
      ptr__ += IMC::serialize(end_z_units, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += IMC::serialize(lradius, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(path_ref, bfr__, size__);
      bfr__ += IMC::deserialize(start_lat, bfr__, size__);
      bfr__ += IMC::deserialize(start_lon, bfr__, size__);
      bfr__ += IMC::deserialize(start_z, bfr__, size__);
      bfr__ += IMC::deserialize(start_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(end_lat, bfr__, size__);
      bfr__ += IMC::deserialize(end_lon, bfr__, size__);
      bfr__ += IMC::deserialize(end_z, bfr__, size__);
      bfr__ += IMC::deserialize(end_z_units, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(lradius, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(path_ref, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_z, bfr__, size__);
      bfr__ += IMC::deserialize(start_z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(end_lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(end_lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(end_z, bfr__, size__);
      bfr__ += IMC::deserialize(end_z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lradius, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DesiredPath::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DesiredPath";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 56;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "path_ref", path_ref, nindent__);
      IMC::toJSON(os__, "start_lat", start_lat, nindent__);
      IMC::toJSON(os__, "start_lon", start_lon, nindent__);
      IMC::toJSON(os__, "start_z", start_z, nindent__);
      IMC::toJSON(os__, "start_z_units", start_z_units, nindent__);
      IMC::toJSON(os__, "end_lat", end_lat, nindent__);
      IMC::toJSON(os__, "end_lon", end_lon, nindent__);
      IMC::toJSON(os__, "end_z", end_z, nindent__);
      IMC::toJSON(os__, "end_z_units", end_z_units, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "lradius", lradius, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
    }
  };
}

#endif
