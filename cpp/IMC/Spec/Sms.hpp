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

#ifndef IMC_SMS_HPP_INCLUDED_
#define IMC_SMS_HPP_INCLUDED_

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
  //! SMS.
  class Sms: public Message
  {
  public:
    //! Number.
    std::string number;
    //! Timeout.
    uint16_t timeout;
    //! Contents.
    std::string contents;

    static uint16_t
    getIdStatic(void)
    {
      return 156;
    }

    static Sms*
    cast(Message* msg__)
    {
      return (Sms*)msg__;
    }

    Sms(void)
    {
      m_header.mgid = Sms::getIdStatic();
      clear();
    }

    Sms*
    clone(void) const
    {
      return new Sms(*this);
    }

    void
    clear(void)
    {
      number.clear();
      timeout = 0;
      contents.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Sms& other__ = static_cast<const Sms&>(msg__);
      if (number != other__.number) return false;
      if (timeout != other__.timeout) return false;
      if (contents != other__.contents) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(number, ptr__);
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += IMC::serialize(contents, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(number, bfr__, size__);
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(contents, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(number, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(contents, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Sms::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Sms";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(number) + IMC::getSerializationSize(contents);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "number", number, nindent__);
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      IMC::toJSON(os__, "contents", contents, nindent__);
    }
  };
}

#endif
