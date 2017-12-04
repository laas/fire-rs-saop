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

#ifndef IMC_DESIREDSPEED_HPP_INCLUDED_
#define IMC_DESIREDSPEED_HPP_INCLUDED_

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
#include "../Spec/ControlCommand.hpp"

namespace IMC
{
  //! Desired Speed.
  class DesiredSpeed: public ControlCommand
  {
  public:
    //! Value.
    double value;
    //! Speed Units.
    uint8_t speed_units;

    static uint16_t
    getIdStatic(void)
    {
      return 402;
    }

    static DesiredSpeed*
    cast(Message* msg__)
    {
      return (DesiredSpeed*)msg__;
    }

    DesiredSpeed(void)
    {
      m_header.mgid = DesiredSpeed::getIdStatic();
      clear();
    }

    DesiredSpeed*
    clone(void) const
    {
      return new DesiredSpeed(*this);
    }

    void
    clear(void)
    {
      value = 0;
      speed_units = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DesiredSpeed& other__ = static_cast<const DesiredSpeed&>(msg__);
      if (value != other__.value) return false;
      if (speed_units != other__.speed_units) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(value, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DesiredSpeed::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DesiredSpeed";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    double
    getValueFP(void) const
    {
      return value;
    }

    void
    setValueFP(double val)
    {
      value = val;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "value", value, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
    }
  };
}

#endif
