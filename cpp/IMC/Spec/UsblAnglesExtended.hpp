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

#ifndef IMC_USBLANGLESEXTENDED_HPP_INCLUDED_
#define IMC_USBLANGLESEXTENDED_HPP_INCLUDED_

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
  //! USBL Angles Extended.
  class UsblAnglesExtended: public Message
  {
  public:
    //! Target.
    std::string target;
    //! Local Bearing.
    float lbearing;
    //! Local Elevation.
    float lelevation;
    //! Bearing.
    float bearing;
    //! Elevation.
    float elevation;
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
      return 898;
    }

    static UsblAnglesExtended*
    cast(Message* msg__)
    {
      return (UsblAnglesExtended*)msg__;
    }

    UsblAnglesExtended(void)
    {
      m_header.mgid = UsblAnglesExtended::getIdStatic();
      clear();
    }

    UsblAnglesExtended*
    clone(void) const
    {
      return new UsblAnglesExtended(*this);
    }

    void
    clear(void)
    {
      target.clear();
      lbearing = 0;
      lelevation = 0;
      bearing = 0;
      elevation = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      accuracy = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblAnglesExtended& other__ = static_cast<const UsblAnglesExtended&>(msg__);
      if (target != other__.target) return false;
      if (lbearing != other__.lbearing) return false;
      if (lelevation != other__.lelevation) return false;
      if (bearing != other__.bearing) return false;
      if (elevation != other__.elevation) return false;
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
      ptr__ += IMC::serialize(lbearing, ptr__);
      ptr__ += IMC::serialize(lelevation, ptr__);
      ptr__ += IMC::serialize(bearing, ptr__);
      ptr__ += IMC::serialize(elevation, ptr__);
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
      bfr__ += IMC::deserialize(lbearing, bfr__, size__);
      bfr__ += IMC::deserialize(lelevation, bfr__, size__);
      bfr__ += IMC::deserialize(bearing, bfr__, size__);
      bfr__ += IMC::deserialize(elevation, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(lbearing, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lelevation, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bearing, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(elevation, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(accuracy, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblAnglesExtended::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblAnglesExtended";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 32;
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
      IMC::toJSON(os__, "lbearing", lbearing, nindent__);
      IMC::toJSON(os__, "lelevation", lelevation, nindent__);
      IMC::toJSON(os__, "bearing", bearing, nindent__);
      IMC::toJSON(os__, "elevation", elevation, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "accuracy", accuracy, nindent__);
    }
  };
}

#endif
