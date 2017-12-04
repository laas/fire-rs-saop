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

#ifndef IMC_TRANSMISSIONREQUEST_HPP_INCLUDED_
#define IMC_TRANSMISSIONREQUEST_HPP_INCLUDED_

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
  //! Transmission Request.
  class TransmissionRequest: public Message
  {
  public:
    //! Communication Mean.
    enum CommunicationMeanEnum
    {
      //! WiFi.
      CMEAN_WIFI = 0,
      //! Acoustic.
      CMEAN_ACOUSTIC = 1,
      //! Satellite.
      CMEAN_SATELLITE = 2,
      //! GSM.
      CMEAN_GSM = 3
    };

    //! Data Mode.
    enum DataModeEnum
    {
      //! Inline Message.
      DMODE_INLINEMSG = 0,
      //! Text.
      DMODE_TEXT = 1,
      //! Raw Data.
      DMODE_RAW = 2
    };

    //! Request Identifier.
    uint16_t req_id;
    //! Communication Mean.
    uint8_t comm_mean;
    //! Destination System.
    std::string destination;
    //! Deadline.
    double deadline;
    //! Data Mode.
    uint8_t data_mode;
    //! Message Data.
    InlineMessage<Message> msg_data;
    //! Text Data.
    std::string txt_data;
    //! Raw Data.
    std::vector<char> raw_data;

    static uint16_t
    getIdStatic(void)
    {
      return 515;
    }

    static TransmissionRequest*
    cast(Message* msg__)
    {
      return (TransmissionRequest*)msg__;
    }

    TransmissionRequest(void)
    {
      m_header.mgid = TransmissionRequest::getIdStatic();
      clear();
      msg_data.setParent(this);
    }

    TransmissionRequest*
    clone(void) const
    {
      return new TransmissionRequest(*this);
    }

    void
    clear(void)
    {
      req_id = 0;
      comm_mean = 0;
      destination.clear();
      deadline = 0;
      data_mode = 0;
      msg_data.clear();
      txt_data.clear();
      raw_data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TransmissionRequest& other__ = static_cast<const TransmissionRequest&>(msg__);
      if (req_id != other__.req_id) return false;
      if (comm_mean != other__.comm_mean) return false;
      if (destination != other__.destination) return false;
      if (deadline != other__.deadline) return false;
      if (data_mode != other__.data_mode) return false;
      if (msg_data != other__.msg_data) return false;
      if (txt_data != other__.txt_data) return false;
      if (raw_data != other__.raw_data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(req_id, ptr__);
      ptr__ += IMC::serialize(comm_mean, ptr__);
      ptr__ += IMC::serialize(destination, ptr__);
      ptr__ += IMC::serialize(deadline, ptr__);
      ptr__ += IMC::serialize(data_mode, ptr__);
      ptr__ += msg_data.serialize(ptr__);
      ptr__ += IMC::serialize(txt_data, ptr__);
      ptr__ += IMC::serialize(raw_data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(comm_mean, bfr__, size__);
      bfr__ += IMC::deserialize(destination, bfr__, size__);
      bfr__ += IMC::deserialize(deadline, bfr__, size__);
      bfr__ += IMC::deserialize(data_mode, bfr__, size__);
      bfr__ += msg_data.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(txt_data, bfr__, size__);
      bfr__ += IMC::deserialize(raw_data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(comm_mean, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(destination, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(deadline, bfr__, size__);
      bfr__ += IMC::deserialize(data_mode, bfr__, size__);
      bfr__ += msg_data.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(txt_data, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(raw_data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TransmissionRequest::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TransmissionRequest";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(destination) + msg_data.getSerializationSize() + IMC::getSerializationSize(txt_data) + IMC::getSerializationSize(raw_data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "req_id", req_id, nindent__);
      IMC::toJSON(os__, "comm_mean", comm_mean, nindent__);
      IMC::toJSON(os__, "destination", destination, nindent__);
      IMC::toJSON(os__, "deadline", deadline, nindent__);
      IMC::toJSON(os__, "data_mode", data_mode, nindent__);
      msg_data.toJSON(os__, "msg_data", nindent__);
      IMC::toJSON(os__, "txt_data", txt_data, nindent__);
      IMC::toJSON(os__, "raw_data", raw_data, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!msg_data.isNull())
      {
        msg_data.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!msg_data.isNull())
      {
        msg_data.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!msg_data.isNull())
      {
        msg_data.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!msg_data.isNull())
      {
        msg_data.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!msg_data.isNull())
      {
        msg_data.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
