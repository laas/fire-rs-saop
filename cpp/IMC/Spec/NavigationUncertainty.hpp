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

#ifndef IMC_NAVIGATIONUNCERTAINTY_HPP_INCLUDED_
#define IMC_NAVIGATIONUNCERTAINTY_HPP_INCLUDED_

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
  //! Navigation Uncertainty.
  class NavigationUncertainty: public Message
  {
  public:
    //! Variance - x Position.
    float x;
    //! Variance - y Position.
    float y;
    //! Variance - z Position.
    float z;
    //! Variance - Roll.
    float phi;
    //! Variance - Pitch.
    float theta;
    //! Variance - Yaw.
    float psi;
    //! Variance - Gyro. Roll Rate.
    float p;
    //! Variance - Gyro. Pitch Rate.
    float q;
    //! Variance - Gyro. Yaw Rate.
    float r;
    //! Variance - Body-Fixed xx Velocity.
    float u;
    //! Variance - Body-Fixed yy Velocity.
    float v;
    //! Variance - Body-Fixed ww Velocity.
    float w;
    //! Variance - Yaw Bias.
    float bias_psi;
    //! Variance - Gyro. Yaw Rate Bias.
    float bias_r;

    static uint16_t
    getIdStatic(void)
    {
      return 354;
    }

    static NavigationUncertainty*
    cast(Message* msg__)
    {
      return (NavigationUncertainty*)msg__;
    }

    NavigationUncertainty(void)
    {
      m_header.mgid = NavigationUncertainty::getIdStatic();
      clear();
    }

    NavigationUncertainty*
    clone(void) const
    {
      return new NavigationUncertainty(*this);
    }

    void
    clear(void)
    {
      x = 0;
      y = 0;
      z = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      p = 0;
      q = 0;
      r = 0;
      u = 0;
      v = 0;
      w = 0;
      bias_psi = 0;
      bias_r = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::NavigationUncertainty& other__ = static_cast<const NavigationUncertainty&>(msg__);
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (phi != other__.phi) return false;
      if (theta != other__.theta) return false;
      if (psi != other__.psi) return false;
      if (p != other__.p) return false;
      if (q != other__.q) return false;
      if (r != other__.r) return false;
      if (u != other__.u) return false;
      if (v != other__.v) return false;
      if (w != other__.w) return false;
      if (bias_psi != other__.bias_psi) return false;
      if (bias_r != other__.bias_r) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(phi, ptr__);
      ptr__ += IMC::serialize(theta, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      ptr__ += IMC::serialize(p, ptr__);
      ptr__ += IMC::serialize(q, ptr__);
      ptr__ += IMC::serialize(r, ptr__);
      ptr__ += IMC::serialize(u, ptr__);
      ptr__ += IMC::serialize(v, ptr__);
      ptr__ += IMC::serialize(w, ptr__);
      ptr__ += IMC::serialize(bias_psi, ptr__);
      ptr__ += IMC::serialize(bias_r, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(phi, bfr__, size__);
      bfr__ += IMC::deserialize(theta, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      bfr__ += IMC::deserialize(p, bfr__, size__);
      bfr__ += IMC::deserialize(q, bfr__, size__);
      bfr__ += IMC::deserialize(r, bfr__, size__);
      bfr__ += IMC::deserialize(u, bfr__, size__);
      bfr__ += IMC::deserialize(v, bfr__, size__);
      bfr__ += IMC::deserialize(w, bfr__, size__);
      bfr__ += IMC::deserialize(bias_psi, bfr__, size__);
      bfr__ += IMC::deserialize(bias_r, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(p, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(q, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(r, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(u, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(w, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bias_psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bias_r, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return NavigationUncertainty::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "NavigationUncertainty";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 56;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "p", p, nindent__);
      IMC::toJSON(os__, "q", q, nindent__);
      IMC::toJSON(os__, "r", r, nindent__);
      IMC::toJSON(os__, "u", u, nindent__);
      IMC::toJSON(os__, "v", v, nindent__);
      IMC::toJSON(os__, "w", w, nindent__);
      IMC::toJSON(os__, "bias_psi", bias_psi, nindent__);
      IMC::toJSON(os__, "bias_r", bias_r, nindent__);
    }
  };
}

#endif
