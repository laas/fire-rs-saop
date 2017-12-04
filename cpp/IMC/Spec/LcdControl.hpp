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

#ifndef IMC_LCDCONTROL_HPP_INCLUDED_
#define IMC_LCDCONTROL_HPP_INCLUDED_

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
  //! LCD Control.
  class LcdControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Turn off display.
      OP_TURN_OFF = 0,
      //! Turn on display.
      OP_TURN_ON = 1,
      //! Clear display.
      OP_CLEAR = 2,
      //! Write Line #0.
      OP_WRITE0 = 3,
      //! Write Line #1.
      OP_WRITE1 = 4
    };

    //! Operation.
    uint8_t op;
    //! Text.
    std::string text;

    static uint16_t
    getIdStatic(void)
    {
      return 307;
    }

    static LcdControl*
    cast(Message* msg__)
    {
      return (LcdControl*)msg__;
    }

    LcdControl(void)
    {
      m_header.mgid = LcdControl::getIdStatic();
      clear();
    }

    LcdControl*
    clone(void) const
    {
      return new LcdControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      text.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LcdControl& other__ = static_cast<const LcdControl&>(msg__);
      if (op != other__.op) return false;
      if (text != other__.text) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(text, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LcdControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LcdControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(text);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "text", text, nindent__);
    }
  };
}

#endif
