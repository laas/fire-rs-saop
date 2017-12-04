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

#ifndef IMC_MANEUVERCONTROLSTATE_HPP_INCLUDED_
#define IMC_MANEUVERCONTROLSTATE_HPP_INCLUDED_

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
  //! Maneuver Control State.
  class ManeuverControlState: public Message
  {
  public:
    //! State.
    enum StateEnum
    {
      //! Maneuver in progress.
      MCS_EXECUTING = 0,
      //! Maneuver completed.
      MCS_DONE = 1,
      //! Maneuver error.
      MCS_ERROR = 2,
      //! Maneuver stopped.
      MCS_STOPPED = 3
    };

    //! State.
    uint8_t state;
    //! Completion Time.
    uint16_t eta;
    //! Info.
    std::string info;

    static uint16_t
    getIdStatic(void)
    {
      return 470;
    }

    static ManeuverControlState*
    cast(Message* msg__)
    {
      return (ManeuverControlState*)msg__;
    }

    ManeuverControlState(void)
    {
      m_header.mgid = ManeuverControlState::getIdStatic();
      clear();
    }

    ManeuverControlState*
    clone(void) const
    {
      return new ManeuverControlState(*this);
    }

    void
    clear(void)
    {
      state = 0;
      eta = 0;
      info.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ManeuverControlState& other__ = static_cast<const ManeuverControlState&>(msg__);
      if (state != other__.state) return false;
      if (eta != other__.eta) return false;
      if (info != other__.info) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(state, ptr__);
      ptr__ += IMC::serialize(eta, ptr__);
      ptr__ += IMC::serialize(info, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::deserialize(eta, bfr__, size__);
      bfr__ += IMC::deserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(state, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(eta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ManeuverControlState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ManeuverControlState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 3;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(info);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "state", state, nindent__);
      IMC::toJSON(os__, "eta", eta, nindent__);
      IMC::toJSON(os__, "info", info, nindent__);
    }
  };
}

#endif
