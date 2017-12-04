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

#ifndef IMC_DESIREDLINEARSTATE_HPP_INCLUDED_
#define IMC_DESIREDLINEARSTATE_HPP_INCLUDED_

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
  //! Desired Linear State.
  class DesiredLinearState: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Value of x is meaningful.
      FL_X = 0x0001,
      //! Value of y is meaningful.
      FL_Y = 0x0002,
      //! Value of z is meaningful.
      FL_Z = 0x0004,
      //! Value of vx is meaningful.
      FL_VX = 0x0008,
      //! Value of vy is meaningful.
      FL_VY = 0x0010,
      //! Value of vz is meaningful.
      FL_VZ = 0x0020,
      //! Value of ax is meaningful.
      FL_AX = 0x0040,
      //! Value of ay is meaningful.
      FL_AY = 0x0080,
      //! Value of az is meaningful.
      FL_AZ = 0x0100
    };

    //! Desired pos in xx.
    double x;
    //! Desired pos in yy.
    double y;
    //! Desired pos in zz.
    double z;
    //! Desired Linear Speed in xx.
    double vx;
    //! Desired Linear Speed in yy.
    double vy;
    //! Desired Linear Speed in zz.
    double vz;
    //! Desired Linear Acceleration in xx.
    double ax;
    //! Desired Linear Acceleration in yy.
    double ay;
    //! Desired Linear Acceleration in zz.
    double az;
    //! Flags.
    uint16_t flags;

    static uint16_t
    getIdStatic(void)
    {
      return 414;
    }

    static DesiredLinearState*
    cast(Message* msg__)
    {
      return (DesiredLinearState*)msg__;
    }

    DesiredLinearState(void)
    {
      m_header.mgid = DesiredLinearState::getIdStatic();
      clear();
    }

    DesiredLinearState*
    clone(void) const
    {
      return new DesiredLinearState(*this);
    }

    void
    clear(void)
    {
      x = 0;
      y = 0;
      z = 0;
      vx = 0;
      vy = 0;
      vz = 0;
      ax = 0;
      ay = 0;
      az = 0;
      flags = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DesiredLinearState& other__ = static_cast<const DesiredLinearState&>(msg__);
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (vx != other__.vx) return false;
      if (vy != other__.vy) return false;
      if (vz != other__.vz) return false;
      if (ax != other__.ax) return false;
      if (ay != other__.ay) return false;
      if (az != other__.az) return false;
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
      ptr__ += IMC::serialize(vx, ptr__);
      ptr__ += IMC::serialize(vy, ptr__);
      ptr__ += IMC::serialize(vz, ptr__);
      ptr__ += IMC::serialize(ax, ptr__);
      ptr__ += IMC::serialize(ay, ptr__);
      ptr__ += IMC::serialize(az, ptr__);
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
      bfr__ += IMC::deserialize(vx, bfr__, size__);
      bfr__ += IMC::deserialize(vy, bfr__, size__);
      bfr__ += IMC::deserialize(vz, bfr__, size__);
      bfr__ += IMC::deserialize(ax, bfr__, size__);
      bfr__ += IMC::deserialize(ay, bfr__, size__);
      bfr__ += IMC::deserialize(az, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(vx, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(vz, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ax, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ay, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(az, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DesiredLinearState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DesiredLinearState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 74;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "vx", vx, nindent__);
      IMC::toJSON(os__, "vy", vy, nindent__);
      IMC::toJSON(os__, "vz", vz, nindent__);
      IMC::toJSON(os__, "ax", ax, nindent__);
      IMC::toJSON(os__, "ay", ay, nindent__);
      IMC::toJSON(os__, "az", az, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
    }
  };
}

#endif
