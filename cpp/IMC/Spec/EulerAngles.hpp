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

#ifndef IMC_EULERANGLES_HPP_INCLUDED_
#define IMC_EULERANGLES_HPP_INCLUDED_

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
  //! Euler Angles.
  class EulerAngles: public Message
  {
  public:
    //! Device Time.
    double time;
    //! Roll Angle.
    double phi;
    //! Pitch Angle.
    double theta;
    //! Yaw Angle (True).
    double psi;
    //! Yaw Angle (Magnetic).
    double psi_magnetic;

    static uint16_t
    getIdStatic(void)
    {
      return 254;
    }

    static EulerAngles*
    cast(Message* msg__)
    {
      return (EulerAngles*)msg__;
    }

    EulerAngles(void)
    {
      m_header.mgid = EulerAngles::getIdStatic();
      clear();
    }

    EulerAngles*
    clone(void) const
    {
      return new EulerAngles(*this);
    }

    void
    clear(void)
    {
      time = 0;
      phi = 0;
      theta = 0;
      psi = 0;
      psi_magnetic = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EulerAngles& other__ = static_cast<const EulerAngles&>(msg__);
      if (time != other__.time) return false;
      if (phi != other__.phi) return false;
      if (theta != other__.theta) return false;
      if (psi != other__.psi) return false;
      if (psi_magnetic != other__.psi_magnetic) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(time, ptr__);
      ptr__ += IMC::serialize(phi, ptr__);
      ptr__ += IMC::serialize(theta, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      ptr__ += IMC::serialize(psi_magnetic, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(time, bfr__, size__);
      bfr__ += IMC::deserialize(phi, bfr__, size__);
      bfr__ += IMC::deserialize(theta, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      bfr__ += IMC::deserialize(psi_magnetic, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(phi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(theta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi_magnetic, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EulerAngles::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EulerAngles";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 40;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "time", time, nindent__);
      IMC::toJSON(os__, "phi", phi, nindent__);
      IMC::toJSON(os__, "theta", theta, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
      IMC::toJSON(os__, "psi_magnetic", psi_magnetic, nindent__);
    }
  };
}

#endif
