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

#ifndef IMC_LEADERSTATE_HPP_INCLUDED_
#define IMC_LEADERSTATE_HPP_INCLUDED_

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
  //! Leader State.
  class LeaderState: public Message
  {
  public:
    //! Action on the leader state.
    enum ActionontheleaderstateEnum
    {
      //! Request.
      OP_REQUEST = 0,
      //! Set.
      OP_SET = 1,
      //! Report.
      OP_REPORT = 2
    };

    //! Group Name.
    std::string group_name;
    //! Action on the leader state.
    uint8_t op;
    //! Latitude (WGS-84).
    double lat;
    //! Longitude (WGS-84).
    double lon;
    //! Height (WGS-84).
    float height;
    //! Offset north.
    float x;
    //! Offset east.
    float y;
    //! Offset down.
    float z;
    //! Rotation over x axis.
    float phi;
    //! Rotation over y axis.
    float theta;
    //! Rotation over z axis.
    float psi;
    //! Ground Velocity X (North).
    float vx;
    //! Ground Velocity Y (East).
    float vy;
    //! Ground Velocity Z (Down).
    float vz;
    //! Angular Velocity in x.
    float p;
    //! Angular Velocity in y.
    float q;
    //! Angular Velocity in z.
    float r;
    //! Stream Velocity X (North).
    float svx;
    //! Stream Velocity Y (East).
    float svy;
    //! Stream Velocity Z (Down).
    float svz;

    static uint16_t
    getIdStatic(void)
    {
      return 563;
    }

    static LeaderState*
    cast(Message* msg__)
    {
      return (LeaderState*)msg__;
    }

    LeaderState(void)
    {
      m_header.mgid = LeaderState::getIdStatic();
      clear();
    }

    LeaderState*
    clone(void) const
    {
      return new LeaderState(*this);
    }

    void
    clear(void)
    {
      group_name.clear();
      op = 0;
      lat = 0;
      lon = 0;
      height = 0;
      x = 0;
      y = 0;
      z = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      vx = 0;
      vy = 0;
      vz = 0;
      p = 0;
      q = 0;
      r = 0;
      svx = 0;
      svy = 0;
      svz = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LeaderState& other__ = static_cast<const LeaderState&>(msg__);
      if (group_name != other__.group_name) return false;
      if (op != other__.op) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (height != other__.height) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (phi != other__.phi) return false;
      if (theta != other__.theta) return false;
      if (psi != other__.psi) return false;
      if (vx != other__.vx) return false;
      if (vy != other__.vy) return false;
      if (vz != other__.vz) return false;
      if (p != other__.p) return false;
      if (q != other__.q) return false;
      if (r != other__.r) return false;
      if (svx != other__.svx) return false;
      if (svy != other__.svy) return false;
      if (svz != other__.svz) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(group_name, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(height, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(phi, ptr__);
      ptr__ += IMC::serialize(theta, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      ptr__ += IMC::serialize(vx, ptr__);
      ptr__ += IMC::serialize(vy, ptr__);
      ptr__ += IMC::serialize(vz, ptr__);
      ptr__ += IMC::serialize(p, ptr__);
      ptr__ += IMC::serialize(q, ptr__);
      ptr__ += IMC::serialize(r, ptr__);
      ptr__ += IMC::serialize(svx, ptr__);
      ptr__ += IMC::serialize(svy, ptr__);
      ptr__ += IMC::serialize(svz, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(group_name, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(height, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(phi, bfr__, size__);
      bfr__ += IMC::deserialize(theta, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      bfr__ += IMC::deserialize(vx, bfr__, size__);
      bfr__ += IMC::deserialize(vy, bfr__, size__);
      bfr__ += IMC::deserialize(vz, bfr__, size__);
      bfr__ += IMC::deserialize(p, bfr__, size__);
      bfr__ += IMC::deserialize(q, bfr__, size__);
      bfr__ += IMC::deserialize(r, bfr__, size__);
      bfr__ += IMC::deserialize(svx, bfr__, size__);
      bfr__ += IMC::deserialize(svy, bfr__, size__);
      bfr__ += IMC::deserialize(svz, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(group_name, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vx, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vz, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(p, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(q, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(r, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(svx, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(svy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(svz, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LeaderState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LeaderState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 81;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(group_name);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "group_name", group_name, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "height", height, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "vx", vx, nindent__);
      IMC::toJSON(os__, "vy", vy, nindent__);
      IMC::toJSON(os__, "vz", vz, nindent__);
      IMC::toJSON(os__, "p", p, nindent__);
      IMC::toJSON(os__, "q", q, nindent__);
      IMC::toJSON(os__, "r", r, nindent__);
      IMC::toJSON(os__, "svx", svx, nindent__);
      IMC::toJSON(os__, "svy", svy, nindent__);
      IMC::toJSON(os__, "svz", svz, nindent__);
    }
  };
}

#endif
