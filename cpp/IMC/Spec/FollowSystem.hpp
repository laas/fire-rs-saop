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

#ifndef IMC_FOLLOWSYSTEM_HPP_INCLUDED_
#define IMC_FOLLOWSYSTEM_HPP_INCLUDED_

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
  //! Follow System.
  class FollowSystem: public Message
  {
  public:
    //! System To Follow.
    uint16_t system;
    //! Duration.
    uint16_t duration;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Offset -- X.
    float x;
    //! Offset -- Y.
    float y;
    //! Coordinate -- Z.
    float z;
    //! Z Units.
    uint8_t z_units;

    static uint16_t
    getIdStatic(void)
    {
      return 471;
    }

    static FollowSystem*
    cast(Message* msg__)
    {
      return (FollowSystem*)msg__;
    }

    FollowSystem(void)
    {
      m_header.mgid = FollowSystem::getIdStatic();
      clear();
    }

    FollowSystem*
    clone(void) const
    {
      return new FollowSystem(*this);
    }

    void
    clear(void)
    {
      system = 0;
      duration = 0;
      speed = 0;
      speed_units = 0;
      x = 0;
      y = 0;
      z = 0;
      z_units = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FollowSystem& other__ = static_cast<const FollowSystem&>(msg__);
      if (system != other__.system) return false;
      if (duration != other__.duration) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (z_units != other__.z_units) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(system, ptr__);
      ptr__ += IMC::serialize(duration, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(system, bfr__, size__);
      bfr__ += IMC::deserialize(duration, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(system, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(duration, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FollowSystem::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FollowSystem";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 22;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "system", system, nindent__);
      IMC::toJSON(os__, "duration", duration, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
    }
  };
}

#endif
