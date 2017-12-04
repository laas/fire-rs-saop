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

#ifndef IMC_SMSSTATE_HPP_INCLUDED_
#define IMC_SMSSTATE_HPP_INCLUDED_

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
  //! SMS State.
  class SmsState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Accepted.
      SMS_ACCEPTED = 0,
      //! Rejected.
      SMS_REJECTED = 1,
      //! Interrupted.
      SMS_INTERRUPTED = 2,
      //! Completed.
      SMS_COMPLETED = 3,
      //! Idle.
      SMS_IDLE = 4,
      //! Transmitting.
      SMS_TRANSMITTING = 5,
      //! Receiving.
      SMS_RECEIVING = 6
    };

    //! Sequence Number.
    uint32_t seq;
    //! State.
    uint8_t state;
    //! Error Message.
    std::string error;

    static uint16_t
    getIdStatic(void)
    {
      return 159;
    }

    static SmsState*
    cast(Message* msg__)
    {
      return (SmsState*)msg__;
    }

    SmsState(void)
    {
      m_header.mgid = SmsState::getIdStatic();
      clear();
    }

    SmsState*
    clone(void) const
    {
      return new SmsState(*this);
    }

    void
    clear(void)
    {
      seq = 0;
      state = 0;
      error.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SmsState& other__ = static_cast<const SmsState&>(msg__);
      if (seq != other__.seq) return false;
      if (state != other__.state) return false;
      if (error != other__.error) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(seq, ptr__);
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(error, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SmsState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SmsState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(error);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "seq", seq, nindent__);
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "error", error, nindent__);
    }
  };
}

#endif
