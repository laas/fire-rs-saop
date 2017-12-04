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

#ifndef IMC_HISTORICDATAQUERY_HPP_INCLUDED_
#define IMC_HISTORICDATAQUERY_HPP_INCLUDED_

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
#include "../Spec/HistoricData.hpp"

namespace IMC
{
  //! Historic Data Query.
  class HistoricDataQuery: public Message
  {
  public:
    //! Request Type.
    enum RequestTypeEnum
    {
      //! Query.
      HRTYPE_QUERY = 1,
      //! Reply.
      HRTYPE_REPLY = 2,
      //! Clear.
      HRTYPE_CLEAR = 3
    };

    //! Request Id.
    uint16_t req_id;
    //! Request Type.
    uint8_t type;
    //! Maximum Size.
    uint16_t max_size;
    //! Data.
    InlineMessage<HistoricData> data;

    static uint16_t
    getIdStatic(void)
    {
      return 187;
    }

    static HistoricDataQuery*
    cast(Message* msg__)
    {
      return (HistoricDataQuery*)msg__;
    }

    HistoricDataQuery(void)
    {
      m_header.mgid = HistoricDataQuery::getIdStatic();
      clear();
      data.setParent(this);
    }

    HistoricDataQuery*
    clone(void) const
    {
      return new HistoricDataQuery(*this);
    }

    void
    clear(void)
    {
      req_id = 0;
      type = 0;
      max_size = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::HistoricDataQuery& other__ = static_cast<const HistoricDataQuery&>(msg__);
      if (req_id != other__.req_id) return false;
      if (type != other__.type) return false;
      if (max_size != other__.max_size) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(req_id, ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(max_size, ptr__);
      ptr__ += data.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(max_size, bfr__, size__);
      bfr__ += data.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(req_id, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_size, bfr__, size__);
      bfr__ += data.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return HistoricDataQuery::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "HistoricDataQuery";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return data.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "req_id", req_id, nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "max_size", max_size, nindent__);
      data.toJSON(os__, "data", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!data.isNull())
      {
        data.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!data.isNull())
      {
        data.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
