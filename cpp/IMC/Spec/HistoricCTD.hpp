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

#ifndef IMC_HISTORICCTD_HPP_INCLUDED_
#define IMC_HISTORICCTD_HPP_INCLUDED_

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
  //! Historic CTD.
  class HistoricCTD: public Message
  {
  public:
    //! Conductivity.
    float conductivity;
    //! Temperature.
    float temperature;
    //! Depth.
    float depth;

    static uint16_t
    getIdStatic(void)
    {
      return 107;
    }

    static HistoricCTD*
    cast(Message* msg__)
    {
      return (HistoricCTD*)msg__;
    }

    HistoricCTD(void)
    {
      m_header.mgid = HistoricCTD::getIdStatic();
      clear();
    }

    HistoricCTD*
    clone(void) const
    {
      return new HistoricCTD(*this);
    }

    void
    clear(void)
    {
      conductivity = 0;
      temperature = 0;
      depth = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::HistoricCTD& other__ = static_cast<const HistoricCTD&>(msg__);
      if (conductivity != other__.conductivity) return false;
      if (temperature != other__.temperature) return false;
      if (depth != other__.depth) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(conductivity, ptr__);
      ptr__ += IMC::serialize(temperature, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(conductivity, bfr__, size__);
      bfr__ += IMC::deserialize(temperature, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(conductivity, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(temperature, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return HistoricCTD::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "HistoricCTD";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "conductivity", conductivity, nindent__);
      IMC::toJSON(os__, "temperature", temperature, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
    }
  };
}

#endif
