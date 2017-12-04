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

#ifndef IMC_DISLODGE_HPP_INCLUDED_
#define IMC_DISLODGE_HPP_INCLUDED_

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
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Dislodge Maneuver.
  class Dislodge: public Maneuver
  {
  public:
    //! Direction.
    enum DirectionEnum
    {
      //! Let the vehicle decide.
      DIR_AUTO = 0,
      //! Attempt to move forward.
      DIR_FORWARD = 1,
      //! Attempt to move backward.
      DIR_BACKWARD = 2
    };

    //! Timeout.
    uint16_t timeout;
    //! RPM.
    float rpm;
    //! Direction.
    uint8_t direction;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 483;
    }

    static Dislodge*
    cast(Message* msg__)
    {
      return (Dislodge*)msg__;
    }

    Dislodge(void)
    {
      m_header.mgid = Dislodge::getIdStatic();
      clear();
    }

    Dislodge*
    clone(void) const
    {
      return new Dislodge(*this);
    }

    void
    clear(void)
    {
      timeout = 0;
      rpm = 0;
      direction = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Dislodge& other__ = static_cast<const Dislodge&>(msg__);
      if (timeout != other__.timeout) return false;
      if (rpm != other__.rpm) return false;
      if (direction != other__.direction) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += IMC::serialize(rpm, ptr__);
      ptr__ += IMC::serialize(direction, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(rpm, bfr__, size__);
      bfr__ += IMC::deserialize(direction, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm, bfr__, size__);
      bfr__ += IMC::deserialize(direction, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Dislodge::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Dislodge";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 7;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      IMC::toJSON(os__, "rpm", rpm, nindent__);
      IMC::toJSON(os__, "direction", direction, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }
  };
}

#endif
