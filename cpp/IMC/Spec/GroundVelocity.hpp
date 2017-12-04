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

#ifndef IMC_GROUNDVELOCITY_HPP_INCLUDED_
#define IMC_GROUNDVELOCITY_HPP_INCLUDED_

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
  //! Ground Velocity.
  class GroundVelocity: public Message
  {
  public:
    //! Validity.
    enum ValidityBits
    {
      //! X component is valid.
      VAL_VEL_X = 0x01,
      //! Y component is valid.
      VAL_VEL_Y = 0x02,
      //! Z component is valid.
      VAL_VEL_Z = 0x04
    };

    //! Validity.
    uint8_t validity;
    //! X.
    double x;
    //! Y.
    double y;
    //! Z.
    double z;

    static uint16_t
    getIdStatic(void)
    {
      return 259;
    }

    static GroundVelocity*
    cast(Message* msg__)
    {
      return (GroundVelocity*)msg__;
    }

    GroundVelocity(void)
    {
      m_header.mgid = GroundVelocity::getIdStatic();
      clear();
    }

    GroundVelocity*
    clone(void) const
    {
      return new GroundVelocity(*this);
    }

    void
    clear(void)
    {
      validity = 0;
      x = 0;
      y = 0;
      z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GroundVelocity& other__ = static_cast<const GroundVelocity&>(msg__);
      if (validity != other__.validity) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(validity, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GroundVelocity::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GroundVelocity";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 25;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "validity", validity, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
    }
  };
}

#endif
