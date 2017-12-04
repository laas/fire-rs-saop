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

#ifndef IMC_TREXCOMMAND_HPP_INCLUDED_
#define IMC_TREXCOMMAND_HPP_INCLUDED_

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
  //! TREX Command.
  class TrexCommand: public Message
  {
  public:
    //! Command.
    enum CommandEnum
    {
      //! Disable TREX.
      OP_DISABLE = 0,
      //! Enable TREX.
      OP_ENABLE = 1,
      //! Post Goal.
      OP_POST_GOAL = 2,
      //! Recall Goal.
      OP_RECALL_GOAL = 3,
      //! Request current plan.
      OP_REQUEST_PLAN = 4,
      //! Report current plan.
      OP_REPORT_PLAN = 5
    };

    //! Command.
    uint8_t command;
    //! Goal Id.
    std::string goal_id;
    //! Goal XML.
    std::string goal_xml;

    static uint16_t
    getIdStatic(void)
    {
      return 652;
    }

    static TrexCommand*
    cast(Message* msg__)
    {
      return (TrexCommand*)msg__;
    }

    TrexCommand(void)
    {
      m_header.mgid = TrexCommand::getIdStatic();
      clear();
    }

    TrexCommand*
    clone(void) const
    {
      return new TrexCommand(*this);
    }

    void
    clear(void)
    {
      command = 0;
      goal_id.clear();
      goal_xml.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TrexCommand& other__ = static_cast<const TrexCommand&>(msg__);
      if (command != other__.command) return false;
      if (goal_id != other__.goal_id) return false;
      if (goal_xml != other__.goal_xml) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(command, ptr__);
      ptr__ += IMC::serialize(goal_id, ptr__);
      ptr__ += IMC::serialize(goal_xml, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::deserialize(goal_id, bfr__, size__);
      bfr__ += IMC::deserialize(goal_xml, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(goal_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(goal_xml, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexCommand::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexCommand";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(goal_id) + IMC::getSerializationSize(goal_xml);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "command", command, nindent__);
      IMC::toJSON(os__, "goal_id", goal_id, nindent__);
      IMC::toJSON(os__, "goal_xml", goal_xml, nindent__);
    }
  };
}

#endif
