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

#ifndef IMC_BUTTONEVENT_HPP_INCLUDED_
#define IMC_BUTTONEVENT_HPP_INCLUDED_

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
  //! Button Event.
  class ButtonEvent: public Message
  {
  public:
    //! Button.
    uint8_t button;
    //! Value.
    uint8_t value;

    static uint16_t
    getIdStatic(void)
    {
      return 306;
    }

    static ButtonEvent*
    cast(Message* msg__)
    {
      return (ButtonEvent*)msg__;
    }

    ButtonEvent(void)
    {
      m_header.mgid = ButtonEvent::getIdStatic();
      clear();
    }

    ButtonEvent*
    clone(void) const
    {
      return new ButtonEvent(*this);
    }

    void
    clear(void)
    {
      button = 0;
      value = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ButtonEvent& other__ = static_cast<const ButtonEvent&>(msg__);
      if (button != other__.button) return false;
      if (value != other__.value) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(button, ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(button, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(button, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ButtonEvent::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ButtonEvent";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    double
    getValueFP(void) const
    {
      return static_cast<double>(value);
    }

    void
    setValueFP(double val)
    {
      value = static_cast<uint8_t>(val);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "button", button, nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
    }
  };
}

#endif
