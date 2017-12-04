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

#ifndef IMC_UASIMULATION_HPP_INCLUDED_
#define IMC_UASIMULATION_HPP_INCLUDED_

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
  //! Underwater Acoustics Simulation.
  class UASimulation: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Data Transmission.
      UAS_DATA = 0,
      //! Ping.
      UAS_PING = 1,
      //! Ping Reply.
      UAS_PING_REPLY = 2
    };

    //! Type.
    uint8_t type;
    //! Transmission Speed.
    uint16_t speed;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 52;
    }

    static UASimulation*
    cast(Message* msg__)
    {
      return (UASimulation*)msg__;
    }

    UASimulation(void)
    {
      m_header.mgid = UASimulation::getIdStatic();
      clear();
    }

    UASimulation*
    clone(void) const
    {
      return new UASimulation(*this);
    }

    void
    clear(void)
    {
      type = 0;
      speed = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UASimulation& other__ = static_cast<const UASimulation&>(msg__);
      if (type != other__.type) return false;
      if (speed != other__.speed) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UASimulation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UASimulation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 3;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
