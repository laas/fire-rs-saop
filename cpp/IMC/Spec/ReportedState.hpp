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

#ifndef IMC_REPORTEDSTATE_HPP_INCLUDED_
#define IMC_REPORTEDSTATE_HPP_INCLUDED_

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
  //! Reported State.
  class ReportedState: public Message
  {
  public:
    //! Source Type.
    enum SourceTypeEnum
    {
      //! Wi-Fi.
      STYPE_WI_FI = 0,
      //! Tracker.
      STYPE_TRACKER = 1,
      //! SMS.
      STYPE_SMS = 2,
      //! Acoustic Modem.
      STYPE_ACOUSTIC_MODEM = 3,
      //! Unknown source.
      STYPE_UNKNOWN = 254
    };

    //! Latitude.
    double lat;
    //! Longitude.
    double lon;
    //! Depth.
    double depth;
    //! Roll.
    double roll;
    //! Pitch.
    double pitch;
    //! Yaw.
    double yaw;
    //! Reception Time.
    double rcp_time;
    //! System Identifier.
    std::string sid;
    //! Source Type.
    uint8_t s_type;

    static uint16_t
    getIdStatic(void)
    {
      return 600;
    }

    static ReportedState*
    cast(Message* msg__)
    {
      return (ReportedState*)msg__;
    }

    ReportedState(void)
    {
      m_header.mgid = ReportedState::getIdStatic();
      clear();
    }

    ReportedState*
    clone(void) const
    {
      return new ReportedState(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      depth = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
      rcp_time = 0;
      sid.clear();
      s_type = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ReportedState& other__ = static_cast<const ReportedState&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (depth != other__.depth) return false;
      if (roll != other__.roll) return false;
      if (pitch != other__.pitch) return false;
      if (yaw != other__.yaw) return false;
      if (rcp_time != other__.rcp_time) return false;
      if (sid != other__.sid) return false;
      if (s_type != other__.s_type) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      ptr__ += IMC::serialize(roll, ptr__);
      ptr__ += IMC::serialize(pitch, ptr__);
      ptr__ += IMC::serialize(yaw, ptr__);
      ptr__ += IMC::serialize(rcp_time, ptr__);
      ptr__ += IMC::serialize(sid, ptr__);
      ptr__ += IMC::serialize(s_type, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(roll, bfr__, size__);
      bfr__ += IMC::deserialize(pitch, bfr__, size__);
      bfr__ += IMC::deserialize(yaw, bfr__, size__);
      bfr__ += IMC::deserialize(rcp_time, bfr__, size__);
      bfr__ += IMC::deserialize(sid, bfr__, size__);
      bfr__ += IMC::deserialize(s_type, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(roll, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pitch, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(yaw, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rcp_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sid, bfr__, size__);
      bfr__ += IMC::deserialize(s_type, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ReportedState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ReportedState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 57;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(sid);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
      IMC::toJSON(os__, "roll", roll, nindent__);
      IMC::toJSON(os__, "pitch", pitch, nindent__);
      IMC::toJSON(os__, "yaw", yaw, nindent__);
      IMC::toJSON(os__, "rcp_time", rcp_time, nindent__);
      IMC::toJSON(os__, "sid", sid, nindent__);
      IMC::toJSON(os__, "s_type", s_type, nindent__);
    }
  };
}

#endif
