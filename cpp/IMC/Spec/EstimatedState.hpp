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

#ifndef IMC_ESTIMATEDSTATE_HPP_INCLUDED_
#define IMC_ESTIMATEDSTATE_HPP_INCLUDED_

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
  //! Estimated State.
  class EstimatedState: public Message
  {
  public:
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
    //! Body-Fixed xx Velocity.
    float u;
    //! Body-Fixed yy Velocity.
    float v;
    //! Body-Fixed zz Velocity.
    float w;
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
    //! Depth.
    float depth;
    //! Altitude.
    float alt;

    static uint16_t
    getIdStatic(void)
    {
      return 350;
    }

    static EstimatedState*
    cast(Message* msg__)
    {
      return (EstimatedState*)msg__;
    }

    EstimatedState(void)
    {
      m_header.mgid = EstimatedState::getIdStatic();
      clear();
    }

    EstimatedState*
    clone(void) const
    {
      return new EstimatedState(*this);
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
      vx = 0;
      vy = 0;
      vz = 0;
      p = 0;
      q = 0;
      r = 0;
      depth = 0;
      alt = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EstimatedState& other__ = static_cast<const EstimatedState&>(msg__);
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
      if (vx != other__.vx) return false;
      if (vy != other__.vy) return false;
      if (vz != other__.vz) return false;
      if (p != other__.p) return false;
      if (q != other__.q) return false;
      if (r != other__.r) return false;
      if (depth != other__.depth) return false;
      if (alt != other__.alt) return false;
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
      ptr__ += IMC::serialize(vx, ptr__);
      ptr__ += IMC::serialize(vy, ptr__);
      ptr__ += IMC::serialize(vz, ptr__);
      ptr__ += IMC::serialize(p, ptr__);
      ptr__ += IMC::serialize(q, ptr__);
      ptr__ += IMC::serialize(r, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      ptr__ += IMC::serialize(alt, ptr__);
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
      bfr__ += IMC::deserialize(vx, bfr__, size__);
      bfr__ += IMC::deserialize(vy, bfr__, size__);
      bfr__ += IMC::deserialize(vz, bfr__, size__);
      bfr__ += IMC::deserialize(p, bfr__, size__);
      bfr__ += IMC::deserialize(q, bfr__, size__);
      bfr__ += IMC::deserialize(r, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(alt, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(vx, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vz, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(p, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(q, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(r, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(alt, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EstimatedState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EstimatedState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 88;
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
      IMC::toJSON(os__, "vx", vx, nindent__);
      IMC::toJSON(os__, "vy", vy, nindent__);
      IMC::toJSON(os__, "vz", vz, nindent__);
      IMC::toJSON(os__, "p", p, nindent__);
      IMC::toJSON(os__, "q", q, nindent__);
      IMC::toJSON(os__, "r", r, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
      IMC::toJSON(os__, "alt", alt, nindent__);
    }
  };
}

#endif
