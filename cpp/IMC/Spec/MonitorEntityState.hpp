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

#ifndef IMC_MONITORENTITYSTATE_HPP_INCLUDED_
#define IMC_MONITORENTITYSTATE_HPP_INCLUDED_

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
  //! Monitor Entity State.
  class MonitorEntityState: public Message
  {
  public:
    //! Command.
    enum CommandEnum
    {
      //! Reset to defaults.
      MES_RESET = 0,
      //! Enable Monitoring.
      MES_ENABLE = 1,
      //! Disable Monitoring.
      MES_DISABLE = 2,
      //! Enable Monitoring (exclusive - disables all others).
      MES_ENABLE_EXCLUSIVE = 3,
      //! Status Report.
      MES_STATUS = 4
    };

    //! Command.
    uint8_t command;
    //! Entity Names.
    std::string entities;

    static uint16_t
    getIdStatic(void)
    {
      return 502;
    }

    static MonitorEntityState*
    cast(Message* msg__)
    {
      return (MonitorEntityState*)msg__;
    }

    MonitorEntityState(void)
    {
      m_header.mgid = MonitorEntityState::getIdStatic();
      clear();
    }

    MonitorEntityState*
    clone(void) const
    {
      return new MonitorEntityState(*this);
    }

    void
    clear(void)
    {
      command = 0;
      entities.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::MonitorEntityState& other__ = static_cast<const MonitorEntityState&>(msg__);
      if (command != other__.command) return false;
      if (entities != other__.entities) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(command, ptr__);
      ptr__ += IMC::serialize(entities, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::deserialize(entities, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(entities, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return MonitorEntityState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "MonitorEntityState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(entities);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "command", command, nindent__);
      IMC::toJSON(os__, "entities", entities, nindent__);
    }
  };
}

#endif
