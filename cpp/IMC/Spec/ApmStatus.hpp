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

#ifndef IMC_APMSTATUS_HPP_INCLUDED_
#define IMC_APMSTATUS_HPP_INCLUDED_

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
  //! APM Status.
  class ApmStatus: public Message
  {
  public:
    //! Severity.
    enum SeverityEnum
    {
      //! Emergency.
      APM_EMERGENCY = 0,
      //! Alert.
      APM_ALERT = 1,
      //! Critical.
      APM_CRITICAL = 2,
      //! Error.
      APM_ERROR = 3,
      //! Warning.
      APM_WARNING = 4,
      //! Notice.
      APM_NOTICE = 5,
      //! Info.
      APM_INFO = 6,
      //! Debug.
      APM_DEBUG = 7
    };

    //! Severity.
    uint8_t severity;
    //! Text.
    std::string text;

    static uint16_t
    getIdStatic(void)
    {
      return 906;
    }

    static ApmStatus*
    cast(Message* msg__)
    {
      return (ApmStatus*)msg__;
    }

    ApmStatus(void)
    {
      m_header.mgid = ApmStatus::getIdStatic();
      clear();
    }

    ApmStatus*
    clone(void) const
    {
      return new ApmStatus(*this);
    }

    void
    clear(void)
    {
      severity = 0;
      text.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ApmStatus& other__ = static_cast<const ApmStatus&>(msg__);
      if (severity != other__.severity) return false;
      if (text != other__.text) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(severity, ptr__);
      ptr__ += IMC::serialize(text, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(severity, bfr__, size__);
      bfr__ += IMC::deserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(severity, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(text, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ApmStatus::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ApmStatus";
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
      IMC::toJSON(os__, "severity", severity, nindent__);
      IMC::toJSON(os__, "text", text, nindent__);
    }
  };
}

#endif
