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

#ifndef IMC_TRANSPORTBINDINGS_HPP_INCLUDED_
#define IMC_TRANSPORTBINDINGS_HPP_INCLUDED_

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
  //! Transport Bindings.
  class TransportBindings: public Message
  {
  public:
    //! Consumer name.
    std::string consumer;
    //! Message Identifier.
    uint16_t message_id;

    static uint16_t
    getIdStatic(void)
    {
      return 8;
    }

    static TransportBindings*
    cast(Message* msg__)
    {
      return (TransportBindings*)msg__;
    }

    TransportBindings(void)
    {
      m_header.mgid = TransportBindings::getIdStatic();
      clear();
    }

    TransportBindings*
    clone(void) const
    {
      return new TransportBindings(*this);
    }

    void
    clear(void)
    {
      consumer.clear();
      message_id = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TransportBindings& other__ = static_cast<const TransportBindings&>(msg__);
      if (consumer != other__.consumer) return false;
      if (message_id != other__.message_id) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(consumer, ptr__);
      ptr__ += IMC::serialize(message_id, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(consumer, bfr__, size__);
      bfr__ += IMC::deserialize(message_id, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(consumer, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(message_id, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TransportBindings::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TransportBindings";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(consumer);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "consumer", consumer, nindent__);
      IMC::toJSON(os__, "message_id", message_id, nindent__);
    }
  };
}

#endif
