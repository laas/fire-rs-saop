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

#ifndef IMC_DESIREDCONTROL_HPP_INCLUDED_
#define IMC_DESIREDCONTROL_HPP_INCLUDED_

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
  //! Desired Control.
  class DesiredControl: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Value of X is meaningful.
      FL_X = 0x01,
      //! Value of Y is meaningful.
      FL_Y = 0x02,
      //! Value of Z is meaningful.
      FL_Z = 0x04,
      //! Value of K is meaningful.
      FL_K = 0x08,
      //! Value of M is meaningful.
      FL_M = 0x10,
      //! Value of N is meaningful.
      FL_N = 0x20
    };

    //! Force along the x axis.
    double x;
    //! Force along the y axis.
    double y;
    //! Force along the z axis.
    double z;
    //! Torque about the x axis.
    double k;
    //! Torque about the y axis.
    double m;
    //! Torque about the z axis.
    double n;
    //! Flags.
    uint8_t flags;

    static uint16_t
    getIdStatic(void)
    {
      return 407;
    }

    static DesiredControl*
    cast(Message* msg__)
    {
      return (DesiredControl*)msg__;
    }

    DesiredControl(void)
    {
      m_header.mgid = DesiredControl::getIdStatic();
      clear();
    }

    DesiredControl*
    clone(void) const
    {
      return new DesiredControl(*this);
    }

    void
    clear(void)
    {
      x = 0;
      y = 0;
      z = 0;
      k = 0;
      m = 0;
      n = 0;
      flags = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DesiredControl& other__ = static_cast<const DesiredControl&>(msg__);
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (k != other__.k) return false;
      if (m != other__.m) return false;
      if (n != other__.n) return false;
      if (flags != other__.flags) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(k, ptr__);
      ptr__ += IMC::serialize(m, ptr__);
      ptr__ += IMC::serialize(n, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(k, bfr__, size__);
      bfr__ += IMC::deserialize(m, bfr__, size__);
      bfr__ += IMC::deserialize(n, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(k, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(m, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(n, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DesiredControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DesiredControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 49;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "k", k, nindent__);
      IMC::toJSON(os__, "m", m, nindent__);
      IMC::toJSON(os__, "n", n, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
    }
  };
}

#endif
