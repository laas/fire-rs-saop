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

#ifndef IMC_ENTITYSTATE_HPP_INCLUDED_
#define IMC_ENTITYSTATE_HPP_INCLUDED_

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
  //! Entity State.
  class EntityState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Bootstrapping.
      ESTA_BOOT = 0,
      //! Normal Operation.
      ESTA_NORMAL = 1,
      //! Fault.
      ESTA_FAULT = 2,
      //! Error.
      ESTA_ERROR = 3,
      //! Failure.
      ESTA_FAILURE = 4
    };

    //! Flags.
    enum FlagsBits
    {
      //! Human Intervention Required.
      EFLA_HUMAN_INTERVENTION = 0x01
    };

    //! State.
    uint8_t state;
    //! Flags.
    uint8_t flags;
    //! Complementary description.
    std::string description;

    static uint16_t
    getIdStatic(void)
    {
      return 1;
    }

    static EntityState*
    cast(Message* msg__)
    {
      return (EntityState*)msg__;
    }

    EntityState(void)
    {
      m_header.mgid = EntityState::getIdStatic();
      clear();
    }

    EntityState*
    clone(void) const
    {
      return new EntityState(*this);
    }

    void
    clear(void)
    {
      state = 0;
      flags = 0;
      description.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::EntityState& other__ = static_cast<const EntityState&>(msg__);
      if (state != other__.state) return false;
      if (flags != other__.flags) return false;
      if (description != other__.description) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += IMC::serialize(description, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::deserialize(description, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(description, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return EntityState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "EntityState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(description);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
      IMC::toJSON(os__, "description", description, nindent__);
    }
  };
}

#endif
