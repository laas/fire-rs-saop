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

#ifndef IMC_UAMTXSTATUS_HPP_INCLUDED_
#define IMC_UAMTXSTATUS_HPP_INCLUDED_

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
  //! UamTxStatus.
  class UamTxStatus: public Message
  {
  public:
    //! Value.
    enum ValueEnum
    {
      //! Transmission Completed.
      UTS_DONE = 0,
      //! Transmission Failed.
      UTS_FAILED = 1,
      //! Transmission Canceled.
      UTS_CANCELED = 2,
      //! Modem is busy.
      UTS_BUSY = 3,
      //! Invalid address.
      UTS_INV_ADDR = 4,
      //! In Progress.
      UTS_IP = 5
    };

    //! Sequence Id.
    uint16_t seq;
    //! Value.
    uint8_t value;
    //! Error Message.
    std::string error;

    static uint16_t
    getIdStatic(void)
    {
      return 816;
    }

    static UamTxStatus*
    cast(Message* msg__)
    {
      return (UamTxStatus*)msg__;
    }

    UamTxStatus(void)
    {
      m_header.mgid = UamTxStatus::getIdStatic();
      clear();
    }

    UamTxStatus*
    clone(void) const
    {
      return new UamTxStatus(*this);
    }

    void
    clear(void)
    {
      seq = 0;
      value = 0;
      error.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UamTxStatus& other__ = static_cast<const UamTxStatus&>(msg__);
      if (seq != other__.seq) return false;
      if (value != other__.value) return false;
      if (error != other__.error) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(seq, ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      ptr__ += IMC::serialize(error, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(seq, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UamTxStatus::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UamTxStatus";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 3;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(error);
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
      IMC::toJSON(os__, "seq", seq, nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
      IMC::toJSON(os__, "error", error, nindent__);
    }
  };
}

#endif
