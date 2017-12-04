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

#ifndef IMC_USBLPOSITIONEXTENDED_HPP_INCLUDED_
#define IMC_USBLPOSITIONEXTENDED_HPP_INCLUDED_

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
  //! USBL Position Extended.
  class UsblPositionExtended: public Message
  {
  public:
    //! Target.
    std::string target;
    //! X.
    float x;
    //! Y.
    float y;
    //! Z.
    float z;
    //! N.
    float n;
    //! E.
    float e;
    //! D.
    float d;
    //! Roll Angle.
    float phi;
    //! Pitch Angle.
    float theta;
    //! Yaw Angle.
    float psi;
    //! Accuracy.
    float accuracy;

    static uint16_t
    getIdStatic(void)
    {
      return 899;
    }

    static UsblPositionExtended*
    cast(Message* msg__)
    {
      return (UsblPositionExtended*)msg__;
    }

    UsblPositionExtended(void)
    {
      m_header.mgid = UsblPositionExtended::getIdStatic();
      clear();
    }

    UsblPositionExtended*
    clone(void) const
    {
      return new UsblPositionExtended(*this);
    }

    void
    clear(void)
    {
      target.clear();
      x = 0;
      y = 0;
      z = 0;
      n = 0;
      e = 0;
      d = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      accuracy = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblPositionExtended& other__ = static_cast<const UsblPositionExtended&>(msg__);
      if (target != other__.target) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (n != other__.n) return false;
      if (e != other__.e) return false;
      if (d != other__.d) return false;
      if (phi != other__.phi) return false;
      if (theta != other__.theta) return false;
      if (psi != other__.psi) return false;
      if (accuracy != other__.accuracy) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(target, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(n, ptr__);
      ptr__ += IMC::serialize(e, ptr__);
      ptr__ += IMC::serialize(d, ptr__);
      ptr__ += IMC::serialize(phi, ptr__);
      ptr__ += IMC::serialize(theta, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      ptr__ += IMC::serialize(accuracy, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(target, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(n, bfr__, size__);
      bfr__ += IMC::deserialize(e, bfr__, size__);
      bfr__ += IMC::deserialize(d, bfr__, size__);
      bfr__ += IMC::deserialize(phi, bfr__, size__);
      bfr__ += IMC::deserialize(theta, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      bfr__ += IMC::deserialize(accuracy, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(target, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(n, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(e, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(d, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(accuracy, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblPositionExtended::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblPositionExtended";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 40;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(target);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "target", target, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "n", n, nindent__);
      IMC::toJSON(os__, "e", e, nindent__);
      IMC::toJSON(os__, "d", d, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "accuracy", accuracy, nindent__);
    }
  };
}

#endif
