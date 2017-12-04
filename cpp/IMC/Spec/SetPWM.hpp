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

#ifndef IMC_SETPWM_HPP_INCLUDED_
#define IMC_SETPWM_HPP_INCLUDED_

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
  //! Set PWM.
  class SetPWM: public Message
  {
  public:
    //! Channel Identifier.
    uint8_t id;
    //! Period.
    uint32_t period;
    //! Duty Cycle.
    uint32_t duty_cycle;

    static uint16_t
    getIdStatic(void)
    {
      return 315;
    }

    static SetPWM*
    cast(Message* msg__)
    {
      return (SetPWM*)msg__;
    }

    SetPWM(void)
    {
      m_header.mgid = SetPWM::getIdStatic();
      clear();
    }

    SetPWM*
    clone(void) const
    {
      return new SetPWM(*this);
    }

    void
    clear(void)
    {
      id = 0;
      period = 0;
      duty_cycle = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SetPWM& other__ = static_cast<const SetPWM&>(msg__);
      if (id != other__.id) return false;
      if (period != other__.period) return false;
      if (duty_cycle != other__.duty_cycle) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(period, ptr__);
      ptr__ += IMC::serialize(duty_cycle, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(period, bfr__, size__);
      bfr__ += IMC::deserialize(duty_cycle, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(period, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(duty_cycle, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SetPWM::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SetPWM";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    uint16_t
    getSubId(void) const
    {
      return id;
    }

    void
    setSubId(uint16_t subid)
    {
      id = (uint8_t)subid;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      IMC::toJSON(os__, "period", period, nindent__);
      IMC::toJSON(os__, "duty_cycle", duty_cycle, nindent__);
    }
  };
}

#endif
