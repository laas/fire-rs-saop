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

#ifndef IMC_SYSTEMGROUP_HPP_INCLUDED_
#define IMC_SYSTEMGROUP_HPP_INCLUDED_

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
  //! System Group.
  class SystemGroup: public Message
  {
  public:
    //! Group List Action.
    enum GroupListActionEnum
    {
      //! Disband.
      OP_Dis = 0,
      //! Set.
      OP_Set = 1,
      //! Request.
      OP_Req = 2,
      //! Change.
      OP_Chg = 3,
      //! Report.
      OP_Rep = 4,
      //! Force.
      OP_Frc = 5
    };

    //! Group Name.
    std::string groupname;
    //! Group List Action.
    uint8_t action;
    //! Systems Name List.
    std::string grouplist;

    static uint16_t
    getIdStatic(void)
    {
      return 181;
    }

    static SystemGroup*
    cast(Message* msg__)
    {
      return (SystemGroup*)msg__;
    }

    SystemGroup(void)
    {
      m_header.mgid = SystemGroup::getIdStatic();
      clear();
    }

    SystemGroup*
    clone(void) const
    {
      return new SystemGroup(*this);
    }

    void
    clear(void)
    {
      groupname.clear();
      action = 0;
      grouplist.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SystemGroup& other__ = static_cast<const SystemGroup&>(msg__);
      if (groupname != other__.groupname) return false;
      if (action != other__.action) return false;
      if (grouplist != other__.grouplist) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(groupname, ptr__);
      ptr__ += IMC::serialize(action, ptr__);
      ptr__ += IMC::serialize(grouplist, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(groupname, bfr__, size__);
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::deserialize(grouplist, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(groupname, bfr__, size__);
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(grouplist, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SystemGroup::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SystemGroup";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(groupname) + IMC::getSerializationSize(grouplist);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "groupname", groupname, nindent__);
      IMC::toJSON(os__, "action", action, nindent__);
      IMC::toJSON(os__, "grouplist", grouplist, nindent__);
    }
  };
}

#endif
