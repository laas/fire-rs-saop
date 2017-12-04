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

#ifndef IMC_GPSFIXRTK_HPP_INCLUDED_
#define IMC_GPSFIXRTK_HPP_INCLUDED_

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
  //! GPS Fix RTK.
  class GpsFixRtk: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! None.
      RTK_NONE = 0x00,
      //! Obs.
      RTK_OBS = 0x01,
      //! Float.
      RTK_FLOAT = 0x02,
      //! Fixed.
      RTK_FIXED = 0x03
    };

    //! Validity.
    enum ValidityBits
    {
      //! Valid Time.
      RFV_VALID_TIME = 0x0001,
      //! Valid Base LLH.
      RFV_VALID_BASE = 0x0002,
      //! Valid Position.
      RFV_VALID_POS = 0x0004,
      //! Valid Velocity.
      RFV_VALID_VEL = 0x0008
    };

    //! Validity.
    uint16_t validity;
    //! Type.
    uint8_t type;
    //! GPS Time of Week.
    uint32_t tow;
    //! Base Latitude WGS-84.
    double base_lat;
    //! Base Longitude WGS-84.
    double base_lon;
    //! Base Height above WGS-84 ellipsoid.
    float base_height;
    //! Position North.
    float n;
    //! Position East.
    float e;
    //! Position Down.
    float d;
    //! Velocity North.
    float v_n;
    //! Velocity East.
    float v_e;
    //! Velocity Down.
    float v_d;
    //! Number of Satellites.
    uint8_t satellites;
    //! IAR Hypotheses.
    uint16_t iar_hyp;
    //! IAR Ratio.
    float iar_ratio;

    static uint16_t
    getIdStatic(void)
    {
      return 293;
    }

    static GpsFixRtk*
    cast(Message* msg__)
    {
      return (GpsFixRtk*)msg__;
    }

    GpsFixRtk(void)
    {
      m_header.mgid = GpsFixRtk::getIdStatic();
      clear();
    }

    GpsFixRtk*
    clone(void) const
    {
      return new GpsFixRtk(*this);
    }

    void
    clear(void)
    {
      validity = 0;
      type = 0;
      tow = 0;
      base_lat = 0;
      base_lon = 0;
      base_height = 0;
      n = 0;
      e = 0;
      d = 0;
      v_n = 0;
      v_e = 0;
      v_d = 0;
      satellites = 0;
      iar_hyp = 0;
      iar_ratio = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GpsFixRtk& other__ = static_cast<const GpsFixRtk&>(msg__);
      if (validity != other__.validity) return false;
      if (type != other__.type) return false;
      if (tow != other__.tow) return false;
      if (base_lat != other__.base_lat) return false;
      if (base_lon != other__.base_lon) return false;
      if (base_height != other__.base_height) return false;
      if (n != other__.n) return false;
      if (e != other__.e) return false;
      if (d != other__.d) return false;
      if (v_n != other__.v_n) return false;
      if (v_e != other__.v_e) return false;
      if (v_d != other__.v_d) return false;
      if (satellites != other__.satellites) return false;
      if (iar_hyp != other__.iar_hyp) return false;
      if (iar_ratio != other__.iar_ratio) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(validity, ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(tow, ptr__);
      ptr__ += IMC::serialize(base_lat, ptr__);
      ptr__ += IMC::serialize(base_lon, ptr__);
      ptr__ += IMC::serialize(base_height, ptr__);
      ptr__ += IMC::serialize(n, ptr__);
      ptr__ += IMC::serialize(e, ptr__);
      ptr__ += IMC::serialize(d, ptr__);
      ptr__ += IMC::serialize(v_n, ptr__);
      ptr__ += IMC::serialize(v_e, ptr__);
      ptr__ += IMC::serialize(v_d, ptr__);
      ptr__ += IMC::serialize(satellites, ptr__);
      ptr__ += IMC::serialize(iar_hyp, ptr__);
      ptr__ += IMC::serialize(iar_ratio, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(tow, bfr__, size__);
      bfr__ += IMC::deserialize(base_lat, bfr__, size__);
      bfr__ += IMC::deserialize(base_lon, bfr__, size__);
      bfr__ += IMC::deserialize(base_height, bfr__, size__);
      bfr__ += IMC::deserialize(n, bfr__, size__);
      bfr__ += IMC::deserialize(e, bfr__, size__);
      bfr__ += IMC::deserialize(d, bfr__, size__);
      bfr__ += IMC::deserialize(v_n, bfr__, size__);
      bfr__ += IMC::deserialize(v_e, bfr__, size__);
      bfr__ += IMC::deserialize(v_d, bfr__, size__);
      bfr__ += IMC::deserialize(satellites, bfr__, size__);
      bfr__ += IMC::deserialize(iar_hyp, bfr__, size__);
      bfr__ += IMC::deserialize(iar_ratio, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(validity, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(tow, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(base_lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(base_lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(base_height, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(n, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(e, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(d, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v_n, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v_e, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v_d, bfr__, size__);
      bfr__ += IMC::deserialize(satellites, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(iar_hyp, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(iar_ratio, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GpsFixRtk::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GpsFixRtk";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 58;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "validity", validity, nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "tow", tow, nindent__);
      IMC::toJSON(os__, "base_lat", base_lat, nindent__);
      IMC::toJSON(os__, "base_lon", base_lon, nindent__);
      IMC::toJSON(os__, "base_height", base_height, nindent__);
      IMC::toJSON(os__, "n", n, nindent__);
      IMC::toJSON(os__, "e", e, nindent__);
      IMC::toJSON(os__, "d", d, nindent__);
      IMC::toJSON(os__, "v_n", v_n, nindent__);
      IMC::toJSON(os__, "v_e", v_e, nindent__);
      IMC::toJSON(os__, "v_d", v_d, nindent__);
      IMC::toJSON(os__, "satellites", satellites, nindent__);
      IMC::toJSON(os__, "iar_hyp", iar_hyp, nindent__);
      IMC::toJSON(os__, "iar_ratio", iar_ratio, nindent__);
    }
  };
}

#endif
