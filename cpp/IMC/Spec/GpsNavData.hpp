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

#ifndef IMC_GPSNAVDATA_HPP_INCLUDED_
#define IMC_GPSNAVDATA_HPP_INCLUDED_

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
  //! GPS Navigation Data.
  class GpsNavData: public Message
  {
  public:
    //! GPS Millisecond Time of Week.
    uint32_t itow;
    //! Latitude.
    double lat;
    //! Longitude.
    double lon;
    //! Height above ellipsoid.
    float height_ell;
    //! Height above sea level.
    float height_sea;
    //! Horizontal Accuracy Estimate.
    float hacc;
    //! Vertical Accuracy Estimate.
    float vacc;
    //! NED North Velocity.
    float vel_n;
    //! NED East Velocity.
    float vel_e;
    //! NED Down Velocity.
    float vel_d;
    //! Speed (3D).
    float speed;
    //! Ground Speed (2D).
    float gspeed;
    //! Heading (2D).
    float heading;
    //! Speed Accuracy Estimate.
    float sacc;
    //! Course / Heading Accuracy Estimate.
    float cacc;

    static uint16_t
    getIdStatic(void)
    {
      return 280;
    }

    static GpsNavData*
    cast(Message* msg__)
    {
      return (GpsNavData*)msg__;
    }

    GpsNavData(void)
    {
      m_header.mgid = GpsNavData::getIdStatic();
      clear();
    }

    GpsNavData*
    clone(void) const
    {
      return new GpsNavData(*this);
    }

    void
    clear(void)
    {
      itow = 0;
      lat = 0;
      lon = 0;
      height_ell = 0;
      height_sea = 0;
      hacc = 0;
      vacc = 0;
      vel_n = 0;
      vel_e = 0;
      vel_d = 0;
      speed = 0;
      gspeed = 0;
      heading = 0;
      sacc = 0;
      cacc = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GpsNavData& other__ = static_cast<const GpsNavData&>(msg__);
      if (itow != other__.itow) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (height_ell != other__.height_ell) return false;
      if (height_sea != other__.height_sea) return false;
      if (hacc != other__.hacc) return false;
      if (vacc != other__.vacc) return false;
      if (vel_n != other__.vel_n) return false;
      if (vel_e != other__.vel_e) return false;
      if (vel_d != other__.vel_d) return false;
      if (speed != other__.speed) return false;
      if (gspeed != other__.gspeed) return false;
      if (heading != other__.heading) return false;
      if (sacc != other__.sacc) return false;
      if (cacc != other__.cacc) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(itow, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(height_ell, ptr__);
      ptr__ += IMC::serialize(height_sea, ptr__);
      ptr__ += IMC::serialize(hacc, ptr__);
      ptr__ += IMC::serialize(vacc, ptr__);
      ptr__ += IMC::serialize(vel_n, ptr__);
      ptr__ += IMC::serialize(vel_e, ptr__);
      ptr__ += IMC::serialize(vel_d, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(gspeed, ptr__);
      ptr__ += IMC::serialize(heading, ptr__);
      ptr__ += IMC::serialize(sacc, ptr__);
      ptr__ += IMC::serialize(cacc, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(itow, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(height_ell, bfr__, size__);
      bfr__ += IMC::deserialize(height_sea, bfr__, size__);
      bfr__ += IMC::deserialize(hacc, bfr__, size__);
      bfr__ += IMC::deserialize(vacc, bfr__, size__);
      bfr__ += IMC::deserialize(vel_n, bfr__, size__);
      bfr__ += IMC::deserialize(vel_e, bfr__, size__);
      bfr__ += IMC::deserialize(vel_d, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(gspeed, bfr__, size__);
      bfr__ += IMC::deserialize(heading, bfr__, size__);
      bfr__ += IMC::deserialize(sacc, bfr__, size__);
      bfr__ += IMC::deserialize(cacc, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(itow, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height_ell, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height_sea, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(hacc, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vacc, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vel_n, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vel_e, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vel_d, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(gspeed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(heading, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sacc, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cacc, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GpsNavData::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GpsNavData";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 68;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "itow", itow, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "height_ell", height_ell, nindent__);
      IMC::toJSON(os__, "height_sea", height_sea, nindent__);
      IMC::toJSON(os__, "hacc", hacc, nindent__);
      IMC::toJSON(os__, "vacc", vacc, nindent__);
      IMC::toJSON(os__, "vel_n", vel_n, nindent__);
      IMC::toJSON(os__, "vel_e", vel_e, nindent__);
      IMC::toJSON(os__, "vel_d", vel_d, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "gspeed", gspeed, nindent__);
      IMC::toJSON(os__, "heading", heading, nindent__);
      IMC::toJSON(os__, "sacc", sacc, nindent__);
      IMC::toJSON(os__, "cacc", cacc, nindent__);
    }
  };
}

#endif
