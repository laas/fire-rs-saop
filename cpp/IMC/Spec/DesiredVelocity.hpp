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

#ifndef IMC_DESIREDVELOCITY_HPP_INCLUDED_
#define IMC_DESIREDVELOCITY_HPP_INCLUDED_

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
  //! Desired Velocity.
  class DesiredVelocity: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Value of u is meaningful.
      FL_SURGE = 0x01,
      //! Value of v is meaningful.
      FL_SWAY = 0x02,
      //! Value of w is meaningful.
      FL_HEAVE = 0x04,
      //! Value of p is meaningful.
      FL_ROLL = 0x08,
      //! Value of q is meaningful.
      FL_PITCH = 0x10,
      //! Value of r is meaningful.
      FL_YAW = 0x20
    };

    //! Desired Linear Speed in xx.
    double u;
    //! Desired Linear Speed in yy.
    double v;
    //! Desired Linear Speed in zz.
    double w;
    //! Desired Angular Speed in xx.
    double p;
    //! Desired Angular Speed in yy.
    double q;
    //! Desired Angular Speed in zz.
    double r;
    //! Flags.
    uint8_t flags;

    static uint16_t
    getIdStatic(void)
    {
      return 409;
    }

    static DesiredVelocity*
    cast(Message* msg__)
    {
      return (DesiredVelocity*)msg__;
    }

    DesiredVelocity(void)
    {
      m_header.mgid = DesiredVelocity::getIdStatic();
      clear();
    }

    DesiredVelocity*
    clone(void) const
    {
      return new DesiredVelocity(*this);
    }

    void
    clear(void)
    {
      u = 0;
      v = 0;
      w = 0;
      p = 0;
      q = 0;
      r = 0;
      flags = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DesiredVelocity& other__ = static_cast<const DesiredVelocity&>(msg__);
      if (u != other__.u) return false;
      if (v != other__.v) return false;
      if (w != other__.w) return false;
      if (p != other__.p) return false;
      if (q != other__.q) return false;
      if (r != other__.r) return false;
      if (flags != other__.flags) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(u, ptr__);
      ptr__ += IMC::serialize(v, ptr__);
      ptr__ += IMC::serialize(w, ptr__);
      ptr__ += IMC::serialize(p, ptr__);
      ptr__ += IMC::serialize(q, ptr__);
      ptr__ += IMC::serialize(r, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(u, bfr__, size__);
      bfr__ += IMC::deserialize(v, bfr__, size__);
      bfr__ += IMC::deserialize(w, bfr__, size__);
      bfr__ += IMC::deserialize(p, bfr__, size__);
      bfr__ += IMC::deserialize(q, bfr__, size__);
      bfr__ += IMC::deserialize(r, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(u, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(v, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(w, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(p, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(q, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(r, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DesiredVelocity::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DesiredVelocity";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 49;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "u", u, nindent__);
      IMC::toJSON(os__, "v", v, nindent__);
      IMC::toJSON(os__, "w", w, nindent__);
      IMC::toJSON(os__, "p", p, nindent__);
      IMC::toJSON(os__, "q", q, nindent__);
      IMC::toJSON(os__, "r", r, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
    }
  };
}

#endif
