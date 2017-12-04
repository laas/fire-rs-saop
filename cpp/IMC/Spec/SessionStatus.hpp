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

#ifndef IMC_SESSIONSTATUS_HPP_INCLUDED_
#define IMC_SESSIONSTATUS_HPP_INCLUDED_

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
  //! Session Status.
  class SessionStatus: public Message
  {
  public:
    //! Status.
    enum StatusEnum
    {
      //! Established.
      STATUS_ESTABLISHED = 1,
      //! Closed.
      STATUS_CLOSED = 2
    };

    //! Session Identifier.
    uint32_t sessid;
    //! Status.
    uint8_t status;

    static uint16_t
    getIdStatic(void)
    {
      return 810;
    }

    static SessionStatus*
    cast(Message* msg__)
    {
      return (SessionStatus*)msg__;
    }

    SessionStatus(void)
    {
      m_header.mgid = SessionStatus::getIdStatic();
      clear();
    }

    SessionStatus*
    clone(void) const
    {
      return new SessionStatus(*this);
    }

    void
    clear(void)
    {
      sessid = 0;
      status = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SessionStatus& other__ = static_cast<const SessionStatus&>(msg__);
      if (sessid != other__.sessid) return false;
      if (status != other__.status) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(sessid, ptr__);
      ptr__ += IMC::serialize(status, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(sessid, bfr__, size__);
      bfr__ += IMC::deserialize(status, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(sessid, bfr__, size__);
      bfr__ += IMC::deserialize(status, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SessionStatus::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SessionStatus";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "sessid", sessid, nindent__);
      IMC::toJSON(os__, "status", status, nindent__);
    }
  };
}

#endif
