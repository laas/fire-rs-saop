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

#ifndef IMC_LOGBOOKENTRY_HPP_INCLUDED_
#define IMC_LOGBOOKENTRY_HPP_INCLUDED_

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
  //! Log Book Entry.
  class LogBookEntry: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Information.
      LBET_INFO = 0,
      //! Warning.
      LBET_WARNING = 1,
      //! Error.
      LBET_ERROR = 2,
      //! Critical.
      LBET_CRITICAL = 3,
      //! Debug.
      LBET_DEBUG = 4
    };

    //! Type.
    uint8_t type;
    //! Timestamp.
    double htime;
    //! Context.
    std::string context;
    //! Text.
    std::string text;

    static uint16_t
    getIdStatic(void)
    {
      return 103;
    }

    static LogBookEntry*
    cast(Message* msg__)
    {
      return (LogBookEntry*)msg__;
    }

    LogBookEntry(void)
    {
      m_header.mgid = LogBookEntry::getIdStatic();
      clear();
    }

    LogBookEntry*
    clone(void) const
    {
      return new LogBookEntry(*this);
    }

    void
    clear(void)
    {
      type = 0;
      htime = 0;
      context.clear();
      text.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LogBookEntry& other__ = static_cast<const LogBookEntry&>(msg__);
      if (type != other__.type) return false;
      if (htime != other__.htime) return false;
      if (context != other__.context) return false;
      if (text != other__.text) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(htime, ptr__);
      ptr__ += IMC::serialize(context, ptr__);
      ptr__ += IMC::serialize(text, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(htime, bfr__, size__);
      bfr__ += IMC::deserialize(context, bfr__, size__);
      bfr__ += IMC::deserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(htime, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(context, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LogBookEntry::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LogBookEntry";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(context) + IMC::getSerializationSize(text);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "htime", htime, nindent__);
      IMC::toJSON(os__, "context", context, nindent__);
      IMC::toJSON(os__, "text", text, nindent__);
    }
  };
}

#endif
