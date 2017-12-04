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

#ifndef IMC_EXTERNALNAVDATA_HPP_INCLUDED_
#define IMC_EXTERNALNAVDATA_HPP_INCLUDED_

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
#include "../Spec/EstimatedState.hpp"

namespace IMC
{
  //! External Navigation Data.
  class ExternalNavData: public Message
  {
  public:
    //! Nav Data Type.
    enum NavDataTypeEnum
    {
      //! Full State.
      EXTNAV_FULL = 0,
      //! Attitude Heading Reference System Only.
      EXTNAV_AHRS = 1,
      //! Position Reference System only.
      EXTNAV_POSREF = 2
    };

    //! Estimated State.
    InlineMessage<EstimatedState> state;
    //! Nav Data Type.
    uint8_t type;

    static uint16_t
    getIdStatic(void)
    {
      return 294;
    }

    static ExternalNavData*
    cast(Message* msg__)
    {
      return (ExternalNavData*)msg__;
    }

    ExternalNavData(void)
    {
      m_header.mgid = ExternalNavData::getIdStatic();
      clear();
      state.setParent(this);
    }

    ExternalNavData*
    clone(void) const
    {
      return new ExternalNavData(*this);
    }

    void
    clear(void)
    {
      state.clear();
      type = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ExternalNavData& other__ = static_cast<const ExternalNavData&>(msg__);
      if (state != other__.state) return false;
      if (type != other__.type) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += state.serialize(ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += state.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += state.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ExternalNavData::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ExternalNavData";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return state.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      state.toJSON(os__, "state", nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!state.isNull())
      {
        state.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!state.isNull())
      {
        state.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!state.isNull())
      {
        state.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!state.isNull())
      {
        state.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!state.isNull())
      {
        state.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
