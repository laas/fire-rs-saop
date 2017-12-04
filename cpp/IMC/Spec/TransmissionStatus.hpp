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

#ifndef IMC_TRANSMISSIONSTATUS_HPP_INCLUDED_
#define IMC_TRANSMISSIONSTATUS_HPP_INCLUDED_

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
  //! Transmission Status.
  class TransmissionStatus: public Message
  {
  public:
    //! Status.
    enum StatusEnum
    {
      //! In progress.
      TSTAT_IN_PROGRESS = 0,
      //! Sent.
      TSTAT_SENT = 1,
      //! Delivered.
      TSTAT_DELIVERED = 51,
      //! Delivery is unknown.
      TSTAT_MAYBE_DELIVERED = 52,
      //! Input Error.
      TSTAT_INPUT_FAILURE = 101,
      //! Temporary Error.
      TSTAT_TEMPORARY_FAILURE = 102,
      //! Permanent Failure.
      TSTAT_PERMANENT_FAILURE = 103
    };

    //! Request Identifier.
    uint16_t req_id;
    //! Status.
    uint8_t status;
    //! Information.
    std::string info;

    static uint16_t
    getIdStatic(void)
    {
      return 516;
    }

    static TransmissionStatus*
    cast(Message* msg__)
    {
      return (TransmissionStatus*)msg__;
    }

    TransmissionStatus(void)
    {
      m_header.mgid = TransmissionStatus::getIdStatic();
      clear();
    }

    TransmissionStatus*
    clone(void) const
    {
      return new TransmissionStatus(*this);
    }

    void
    clear(void)
    {
      req_id = 0;
      status = 0;
      info.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TransmissionStatus& other__ = static_cast<const TransmissionStatus&>(msg__);
      if (req_id != other__.req_id) return false;
      if (status != other__.status) return false;
      if (info != other__.info) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(req_id, ptr__);
      ptr__ += IMC::serialize(status, ptr__);
      ptr__ += IMC::serialize(info, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(status, bfr__, size__);
      bfr__ += IMC::deserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(status, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TransmissionStatus::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TransmissionStatus";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 3;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(info);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "req_id", req_id, nindent__);
      IMC::toJSON(os__, "status", status, nindent__);
      IMC::toJSON(os__, "info", info, nindent__);
    }
  };
}

#endif
