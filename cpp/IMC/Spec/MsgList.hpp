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

#ifndef IMC_MSGLIST_HPP_INCLUDED_
#define IMC_MSGLIST_HPP_INCLUDED_

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
  //! Message List.
  class MsgList: public Message
  {
  public:
    //! Messages.
    MessageList<Message> msgs;

    static uint16_t
    getIdStatic(void)
    {
      return 20;
    }

    static MsgList*
    cast(Message* msg__)
    {
      return (MsgList*)msg__;
    }

    MsgList(void)
    {
      m_header.mgid = MsgList::getIdStatic();
      clear();
      msgs.setParent(this);
    }

    MsgList*
    clone(void) const
    {
      return new MsgList(*this);
    }

    void
    clear(void)
    {
      msgs.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::MsgList& other__ = static_cast<const MsgList&>(msg__);
      if (msgs != other__.msgs) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += msgs.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += msgs.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += msgs.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return MsgList::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "MsgList";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return msgs.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      msgs.toJSON(os__, "msgs", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      msgs.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      msgs.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      msgs.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      msgs.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      msgs.setDestinationEntity(value__);
    }
  };
}

#endif
