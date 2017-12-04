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

#ifndef IMC_GPSFIX_HPP_INCLUDED_
#define IMC_GPSFIX_HPP_INCLUDED_

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
  //! GPS Fix.
  class GpsFix: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Stand Alone.
      GFT_STANDALONE = 0x00,
      //! Differential.
      GFT_DIFFERENTIAL = 0x01,
      //! Dead Reckoning.
      GFT_DEAD_RECKONING = 0x02,
      //! Manual Input.
      GFT_MANUAL_INPUT = 0x03,
      //! Simulation.
      GFT_SIMULATION = 0x04
    };

    //! Validity.
    enum ValidityBits
    {
      //! Valid Date.
      GFV_VALID_DATE = 0x0001,
      //! Valid Time.
      GFV_VALID_TIME = 0x0002,
      //! Valid Position.
      GFV_VALID_POS = 0x0004,
      //! Valid Course Over Ground.
      GFV_VALID_COG = 0x0008,
      //! Valid Speed Over Ground.
      GFV_VALID_SOG = 0x0010,
      //! Valid Horizontal Accuracy Estimate.
      GFV_VALID_HACC = 0x0020,
      //! Valid Vertical Accuracy Estimate.
      GFV_VALID_VACC = 0x0040,
      //! Valid Horizontal Dilution of Precision.
      GFV_VALID_HDOP = 0x0080,
      //! Valid Vertical Dilution of Precision.
      GFV_VALID_VDOP = 0x0100
    };

    //! Validity.
    uint16_t validity;
    //! Type.
    uint8_t type;
    //! UTC Year.
    uint16_t utc_year;
    //! UTC Month.
    uint8_t utc_month;
    //! UTC Day.
    uint8_t utc_day;
    //! UTC Time of Fix.
    float utc_time;
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Height above WGS-84 ellipsoid.
    float height;
    //! Number of Satellites.
    uint8_t satellites;
    //! Course Over Ground.
    float cog;
    //! Speed Over Ground.
    float sog;
    //! Horizontal Dilution of Precision.
    float hdop;
    //! Vertical Dilution of Precision.
    float vdop;
    //! Horizontal Accuracy Estimate.
    float hacc;
    //! Vertical Accuracy Estimate.
    float vacc;

    static uint16_t
    getIdStatic(void)
    {
      return 253;
    }

    static GpsFix*
    cast(Message* msg__)
    {
      return (GpsFix*)msg__;
    }

    GpsFix(void)
    {
      m_header.mgid = GpsFix::getIdStatic();
      clear();
    }

    GpsFix*
    clone(void) const
    {
      return new GpsFix(*this);
    }

    void
    clear(void)
    {
      validity = 0;
      type = 0;
      utc_year = 0;
      utc_month = 0;
      utc_day = 0;
      utc_time = 0;
      lat = 0;
      lon = 0;
      height = 0;
      satellites = 0;
      cog = 0;
      sog = 0;
      hdop = 0;
      vdop = 0;
      hacc = 0;
      vacc = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GpsFix& other__ = static_cast<const GpsFix&>(msg__);
      if (validity != other__.validity) return false;
      if (type != other__.type) return false;
      if (utc_year != other__.utc_year) return false;
      if (utc_month != other__.utc_month) return false;
      if (utc_day != other__.utc_day) return false;
      if (utc_time != other__.utc_time) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (height != other__.height) return false;
      if (satellites != other__.satellites) return false;
      if (cog != other__.cog) return false;
      if (sog != other__.sog) return false;
      if (hdop != other__.hdop) return false;
      if (vdop != other__.vdop) return false;
      if (hacc != other__.hacc) return false;
      if (vacc != other__.vacc) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(validity, ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(utc_year, ptr__);
      ptr__ += IMC::serialize(utc_month, ptr__);
      ptr__ += IMC::serialize(utc_day, ptr__);
      ptr__ += IMC::serialize(utc_time, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(height, ptr__);
      ptr__ += IMC::serialize(satellites, ptr__);
      ptr__ += IMC::serialize(cog, ptr__);
      ptr__ += IMC::serialize(sog, ptr__);
      ptr__ += IMC::serialize(hdop, ptr__);
      ptr__ += IMC::serialize(vdop, ptr__);
      ptr__ += IMC::serialize(hacc, ptr__);
      ptr__ += IMC::serialize(vacc, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(utc_year, bfr__, size__);
      bfr__ += IMC::deserialize(utc_month, bfr__, size__);
      bfr__ += IMC::deserialize(utc_day, bfr__, size__);
      bfr__ += IMC::deserialize(utc_time, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(height, bfr__, size__);
      bfr__ += IMC::deserialize(satellites, bfr__, size__);
      bfr__ += IMC::deserialize(cog, bfr__, size__);
      bfr__ += IMC::deserialize(sog, bfr__, size__);
      bfr__ += IMC::deserialize(hdop, bfr__, size__);
      bfr__ += IMC::deserialize(vdop, bfr__, size__);
      bfr__ += IMC::deserialize(hacc, bfr__, size__);
      bfr__ += IMC::deserialize(vacc, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(validity, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(utc_year, bfr__, size__);
      bfr__ += IMC::deserialize(utc_month, bfr__, size__);
      bfr__ += IMC::deserialize(utc_day, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(utc_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height, bfr__, size__);
      bfr__ += IMC::deserialize(satellites, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cog, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sog, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(hdop, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vdop, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(hacc, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vacc, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GpsFix::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GpsFix";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 56;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "validity", validity, nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "utc_year", utc_year, nindent__);
      IMC::toJSON(os__, "utc_month", utc_month, nindent__);
      IMC::toJSON(os__, "utc_day", utc_day, nindent__);
      IMC::toJSON(os__, "utc_time", utc_time, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "height", height, nindent__);
      IMC::toJSON(os__, "satellites", satellites, nindent__);
      IMC::toJSON(os__, "cog", cog, nindent__);
      IMC::toJSON(os__, "sog", sog, nindent__);
      IMC::toJSON(os__, "hdop", hdop, nindent__);
      IMC::toJSON(os__, "vdop", vdop, nindent__);
      IMC::toJSON(os__, "hacc", hacc, nindent__);
      IMC::toJSON(os__, "vacc", vacc, nindent__);
    }
  };
}

#endif
