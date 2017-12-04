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

#ifndef IMC_REMOTECOMMAND_HPP_INCLUDED_
#define IMC_REMOTECOMMAND_HPP_INCLUDED_

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
  //! Remote Command.
  class RemoteCommand: public RemoteData
  {
  public:
    //! Original Source.
    uint16_t original_source;
    //! Destination.
    uint16_t destination;
    //! Timeout.
    double timeout;
    //! Command.
    InlineMessage<Message> cmd;

    static uint16_t
    getIdStatic(void)
    {
      return 188;
    }

    static RemoteCommand*
    cast(Message* msg__)
    {
      return (RemoteCommand*)msg__;
    }

    RemoteCommand(void)
    {
      m_header.mgid = RemoteCommand::getIdStatic();
      clear();
      cmd.setParent(this);
    }

    RemoteCommand*
    clone(void) const
    {
      return new RemoteCommand(*this);
    }

    void
    clear(void)
    {
      original_source = 0;
      destination = 0;
      timeout = 0;
      cmd.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RemoteCommand& other__ = static_cast<const RemoteCommand&>(msg__);
      if (original_source != other__.original_source) return false;
      if (destination != other__.destination) return false;
      if (timeout != other__.timeout) return false;
      if (cmd != other__.cmd) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(original_source, ptr__);
      ptr__ += IMC::serialize(destination, ptr__);
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += cmd.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(original_source, bfr__, size__);
      bfr__ += IMC::deserialize(destination, bfr__, size__);
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += cmd.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(original_source, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(destination, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += cmd.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RemoteCommand::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RemoteCommand";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return cmd.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "original_source", original_source, nindent__);
      IMC::toJSON(os__, "destination", destination, nindent__);
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      cmd.toJSON(os__, "cmd", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!cmd.isNull())
      {
        cmd.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!cmd.isNull())
      {
        cmd.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!cmd.isNull())
      {
        cmd.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!cmd.isNull())
      {
        cmd.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!cmd.isNull())
      {
        cmd.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
