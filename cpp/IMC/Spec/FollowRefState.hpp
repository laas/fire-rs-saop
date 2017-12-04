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

#ifndef IMC_FOLLOWREFSTATE_HPP_INCLUDED_
#define IMC_FOLLOWREFSTATE_HPP_INCLUDED_

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
#include "../Spec/Reference.hpp"

namespace IMC
{
  //! Follow Reference State.
  class FollowRefState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Waiting for first reference.
      FR_WAIT = 1,
      //! Going towards received reference.
      FR_GOTO = 2,
      //! Loitering after arriving at the reference.
      FR_LOITER = 3,
      //! Hovering after arriving at the reference.
      FR_HOVER = 4,
      //! Moving in z after arriving at the target cylinder.
      FR_ELEVATOR = 5,
      //! Controlling system timed out.
      FR_TIMEOUT = 6
    };

    //! Proximity.
    enum ProximityBits
    {
      //! Far from the destination.
      PROX_FAR = 0x01,
      //! Near in the horizontal plane.
      PROX_XY_NEAR = 0x02,
      //! Near in the vertical plane.
      PROX_Z_NEAR = 0x04
    };

    //! Controlling Source.
    uint16_t control_src;
    //! Controlling Entity.
    uint8_t control_ent;
    //! Reference.
    InlineMessage<Reference> reference;
    //! State.
    uint8_t state;
    //! Proximity.
    uint8_t proximity;

    static uint16_t
    getIdStatic(void)
    {
      return 480;
    }

    static FollowRefState*
    cast(Message* msg__)
    {
      return (FollowRefState*)msg__;
    }

    FollowRefState(void)
    {
      m_header.mgid = FollowRefState::getIdStatic();
      clear();
      reference.setParent(this);
    }

    FollowRefState*
    clone(void) const
    {
      return new FollowRefState(*this);
    }

    void
    clear(void)
    {
      control_src = 0;
      control_ent = 0;
      reference.clear();
      state = 0;
      proximity = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FollowRefState& other__ = static_cast<const FollowRefState&>(msg__);
      if (control_src != other__.control_src) return false;
      if (control_ent != other__.control_ent) return false;
      if (reference != other__.reference) return false;
      if (state != other__.state) return false;
      if (proximity != other__.proximity) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(control_src, ptr__);
      ptr__ += IMC::serialize(control_ent, ptr__);
      ptr__ += reference.serialize(ptr__);
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(proximity, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += reference.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(proximity, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += reference.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(proximity, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FollowRefState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FollowRefState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return reference.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "control_src", control_src, nindent__);
      IMC::toJSON(os__, "control_ent", control_ent, nindent__);
      reference.toJSON(os__, "reference", nindent__);
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "proximity", proximity, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!reference.isNull())
      {
        reference.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!reference.isNull())
      {
        reference.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!reference.isNull())
      {
        reference.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!reference.isNull())
      {
        reference.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!reference.isNull())
      {
        reference.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
