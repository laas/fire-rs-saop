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

#ifndef IMC_SETLEDBRIGHTNESS_HPP_INCLUDED_
#define IMC_SETLEDBRIGHTNESS_HPP_INCLUDED_

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
  //! Set LED Brightness.
  class SetLedBrightness: public Message
  {
  public:
    //! Name.
    std::string name;
    //! Value.
    uint8_t value;

    static uint16_t
    getIdStatic(void)
    {
      return 314;
    }

    static SetLedBrightness*
    cast(Message* msg__)
    {
      return (SetLedBrightness*)msg__;
    }

    SetLedBrightness(void)
    {
      m_header.mgid = SetLedBrightness::getIdStatic();
      clear();
    }

    SetLedBrightness*
    clone(void) const
    {
      return new SetLedBrightness(*this);
    }

    void
    clear(void)
    {
      name.clear();
      value = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SetLedBrightness& other__ = static_cast<const SetLedBrightness&>(msg__);
      if (name != other__.name) return false;
      if (value != other__.value) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(name, ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SetLedBrightness::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SetLedBrightness";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name);
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
      IMC::toJSON(os__, "name", name, nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
    }
  };
}

#endif
