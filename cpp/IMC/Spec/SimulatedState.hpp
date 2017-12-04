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

#ifndef IMC_SIMULATEDSTATE_HPP_INCLUDED_
#define IMC_SIMULATEDSTATE_HPP_INCLUDED_

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
  //! Simulated State.
  class SimulatedState: public Message
  {
  public:
    //! Latitude (WGS-84).
    double lat;
    //! Longitude (WGS-84).
    double lon;
    //! Height (WGS-84).
    float height;
    //! Offset north (m).
    float x;
    //! Offset east (m).
    float y;
    //! Offset down (m).
    float z;
    //! Rotation over x axis.
    float phi;
    //! Rotation over y axis.
    float theta;
    //! Rotation over z axis.
    float psi;
    //! Body-Fixed xx Linear Velocity.
    float u;
    //! Body-Fixed yy Linear Velocity.
    float v;
    //! Body-Fixed zz Linear Velocity.
    float w;
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
      return 50;
    }

    static SimulatedState*
    cast(Message* msg__)
    {
      return (SimulatedState*)msg__;
    }

    SimulatedState(void)
    {
      m_header.mgid = SimulatedState::getIdStatic();
      clear();
    }

    SimulatedState*
    clone(void) const
    {
      return new SimulatedState(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      height = 0;
      x = 0;
      y = 0;
      z = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      u = 0;
      v = 0;
      w = 0;
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
      const IMC::SimulatedState& other__ = static_cast<const SimulatedState&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (height != other__.height) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (phi != other__.phi) return false;
      if (theta != other__.theta) return false;
      if (psi != other__.psi) return false;
      if (u != other__.u) return false;
      if (v != other__.v) return false;
      if (w != other__.w) return false;
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
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(height, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(phi, ptr__);
      ptr__ += IMC::serialize(theta, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      ptr__ += IMC::serialize(u, ptr__);
      ptr__ += IMC::serialize(v, ptr__);
      ptr__ += IMC::serialize(w, ptr__);
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
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(height, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(phi, bfr__, size__);
      bfr__ += IMC::deserialize(theta, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      bfr__ += IMC::deserialize(u, bfr__, size__);
      bfr__ += IMC::deserialize(v, bfr__, size__);
      bfr__ += IMC::deserialize(w, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(u, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(w, bfr__, size__);
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
      return SimulatedState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SimulatedState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 80;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "height", height, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "u", u, nindent__);
      IMC::toJSON(os__, "v", v, nindent__);
      IMC::toJSON(os__, "w", w, nindent__);
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
