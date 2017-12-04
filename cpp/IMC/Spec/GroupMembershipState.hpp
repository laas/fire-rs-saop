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

#ifndef IMC_GROUPMEMBERSHIPSTATE_HPP_INCLUDED_
#define IMC_GROUPMEMBERSHIPSTATE_HPP_INCLUDED_

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
  //! Group Membership State.
  class GroupMembershipState: public Message
  {
  public:
    //! Group Name.
    std::string group_name;
    //! Communication Links Assertion.
    uint32_t links;

    static uint16_t
    getIdStatic(void)
    {
      return 180;
    }

    static GroupMembershipState*
    cast(Message* msg__)
    {
      return (GroupMembershipState*)msg__;
    }

    GroupMembershipState(void)
    {
      m_header.mgid = GroupMembershipState::getIdStatic();
      clear();
    }

    GroupMembershipState*
    clone(void) const
    {
      return new GroupMembershipState(*this);
    }

    void
    clear(void)
    {
      group_name.clear();
      links = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::GroupMembershipState& other__ = static_cast<const GroupMembershipState&>(msg__);
      if (group_name != other__.group_name) return false;
      if (links != other__.links) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(group_name, ptr__);
      ptr__ += IMC::serialize(links, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(group_name, bfr__, size__);
      bfr__ += IMC::deserialize(links, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(group_name, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(links, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return GroupMembershipState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "GroupMembershipState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(group_name);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "group_name", group_name, nindent__);
      IMC::toJSON(os__, "links", links, nindent__);
    }
  };
}

#endif
