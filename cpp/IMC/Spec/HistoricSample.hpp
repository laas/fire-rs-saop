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

#ifndef IMC_HISTORICSAMPLE_HPP_INCLUDED_
#define IMC_HISTORICSAMPLE_HPP_INCLUDED_

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
#include "../Spec/RemoteData.hpp"

namespace IMC
{
  //! Historic Data Sample.
  class HistoricSample: public RemoteData
  {
  public:
    //! Original System Id.
    uint16_t sys_id;
    //! Priority.
    int8_t priority;
    //! X offset.
    int16_t x;
    //! Y offset.
    int16_t y;
    //! Z offset.
    int16_t z;
    //! Time offset.
    int16_t t;
    //! Data Sample.
    InlineMessage<Message> sample;

    static uint16_t
    getIdStatic(void)
    {
      return 186;
    }

    static HistoricSample*
    cast(Message* msg__)
    {
      return (HistoricSample*)msg__;
    }

    HistoricSample(void)
    {
      m_header.mgid = HistoricSample::getIdStatic();
      clear();
      sample.setParent(this);
    }

    HistoricSample*
    clone(void) const
    {
      return new HistoricSample(*this);
    }

    void
    clear(void)
    {
      sys_id = 0;
      priority = 0;
      x = 0;
      y = 0;
      z = 0;
      t = 0;
      sample.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::HistoricSample& other__ = static_cast<const HistoricSample&>(msg__);
      if (sys_id != other__.sys_id) return false;
      if (priority != other__.priority) return false;
      if (x != other__.x) return false;
      if (y != other__.y) return false;
      if (z != other__.z) return false;
      if (t != other__.t) return false;
      if (sample != other__.sample) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(sys_id, ptr__);
      ptr__ += IMC::serialize(priority, ptr__);
      ptr__ += IMC::serialize(x, ptr__);
      ptr__ += IMC::serialize(y, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(t, ptr__);
      ptr__ += sample.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(sys_id, bfr__, size__);
      bfr__ += IMC::deserialize(priority, bfr__, size__);
      bfr__ += IMC::deserialize(x, bfr__, size__);
      bfr__ += IMC::deserialize(y, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(t, bfr__, size__);
      bfr__ += sample.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(sys_id, bfr__, size__);
      bfr__ += IMC::deserialize(priority, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(t, bfr__, size__);
      bfr__ += sample.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return HistoricSample::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "HistoricSample";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 11;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return sample.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "sys_id", sys_id, nindent__);
      IMC::toJSON(os__, "priority", priority, nindent__);
      IMC::toJSON(os__, "x", x, nindent__);
      IMC::toJSON(os__, "y", y, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "t", t, nindent__);
      sample.toJSON(os__, "sample", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!sample.isNull())
      {
        sample.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!sample.isNull())
      {
        sample.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!sample.isNull())
      {
        sample.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!sample.isNull())
      {
        sample.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!sample.isNull())
      {
        sample.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
