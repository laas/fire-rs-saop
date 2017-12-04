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

#ifndef IMC_WINDSPEED_HPP_INCLUDED_
#define IMC_WINDSPEED_HPP_INCLUDED_

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
  //! Wind Speed.
  class WindSpeed: public Message
  {
  public:
    //! Direction.
    float direction;
    //! Speed.
    float speed;
    //! Turbulence.
    float turbulence;

    static uint16_t
    getIdStatic(void)
    {
      return 271;
    }

    static WindSpeed*
    cast(Message* msg__)
    {
      return (WindSpeed*)msg__;
    }

    WindSpeed(void)
    {
      m_header.mgid = WindSpeed::getIdStatic();
      clear();
    }

    WindSpeed*
    clone(void) const
    {
      return new WindSpeed(*this);
    }

    void
    clear(void)
    {
      direction = 0;
      speed = 0;
      turbulence = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::WindSpeed& other__ = static_cast<const WindSpeed&>(msg__);
      if (direction != other__.direction) return false;
      if (speed != other__.speed) return false;
      if (turbulence != other__.turbulence) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(direction, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(turbulence, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(direction, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(turbulence, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(direction, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(turbulence, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return WindSpeed::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "WindSpeed";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "direction", direction, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "turbulence", turbulence, nindent__);
    }
  };
}

#endif
