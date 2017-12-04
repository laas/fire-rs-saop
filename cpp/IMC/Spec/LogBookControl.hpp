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

#ifndef IMC_LOGBOOKCONTROL_HPP_INCLUDED_
#define IMC_LOGBOOKCONTROL_HPP_INCLUDED_

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
#include "../Spec/LogBookEntry.hpp"

namespace IMC
{
  //! Log Book Control.
  class LogBookControl: public Message
  {
  public:
    //! Command.
    enum CommandEnum
    {
      //! Get.
      LBC_GET = 0,
      //! Clear.
      LBC_CLEAR = 1,
      //! Get Errors.
      LBC_GET_ERR = 2,
      //! Reply.
      LBC_REPLY = 3
    };

    //! Command.
    uint8_t command;
    //! Timestamp.
    double htime;
    //! Messages.
    MessageList<LogBookEntry> msg;

    static uint16_t
    getIdStatic(void)
    {
      return 104;
    }

    static LogBookControl*
    cast(Message* msg__)
    {
      return (LogBookControl*)msg__;
    }

    LogBookControl(void)
    {
      m_header.mgid = LogBookControl::getIdStatic();
      clear();
      msg.setParent(this);
    }

    LogBookControl*
    clone(void) const
    {
      return new LogBookControl(*this);
    }

    void
    clear(void)
    {
      command = 0;
      htime = 0;
      msg.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LogBookControl& other__ = static_cast<const LogBookControl&>(msg__);
      if (command != other__.command) return false;
      if (htime != other__.htime) return false;
      if (msg != other__.msg) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(command, ptr__);
      ptr__ += IMC::serialize(htime, ptr__);
      ptr__ += msg.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::deserialize(htime, bfr__, size__);
      bfr__ += msg.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(htime, bfr__, size__);
      bfr__ += msg.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LogBookControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LogBookControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return msg.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "command", command, nindent__);
      IMC::toJSON(os__, "htime", htime, nindent__);
      msg.toJSON(os__, "msg", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      msg.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      msg.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      msg.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      msg.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      msg.setDestinationEntity(value__);
    }
  };
}

#endif
