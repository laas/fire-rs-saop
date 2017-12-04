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

#ifndef IMC_AUTOPILOTMODE_HPP_INCLUDED_
#define IMC_AUTOPILOTMODE_HPP_INCLUDED_

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
  //! Autopilot Mode.
  class AutopilotMode: public Message
  {
  public:
    //! Autonomy Level.
    enum AutonomyLevelEnum
    {
      //! Manual.
      AL_MANUAL = 0,
      //! Assisted.
      AL_ASSISTED = 1,
      //! Auto.
      AL_AUTO = 2
    };

    //! Autonomy Level.
    uint8_t autonomy;
    //! Mode.
    std::string mode;

    static uint16_t
    getIdStatic(void)
    {
      return 511;
    }

    static AutopilotMode*
    cast(Message* msg__)
    {
      return (AutopilotMode*)msg__;
    }

    AutopilotMode(void)
    {
      m_header.mgid = AutopilotMode::getIdStatic();
      clear();
    }

    AutopilotMode*
    clone(void) const
    {
      return new AutopilotMode(*this);
    }

    void
    clear(void)
    {
      autonomy = 0;
      mode.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AutopilotMode& other__ = static_cast<const AutopilotMode&>(msg__);
      if (autonomy != other__.autonomy) return false;
      if (mode != other__.mode) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(autonomy, ptr__);
      ptr__ += IMC::serialize(mode, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(autonomy, bfr__, size__);
      bfr__ += IMC::deserialize(mode, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(autonomy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(mode, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AutopilotMode::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AutopilotMode";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(mode);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "autonomy", autonomy, nindent__);
      IMC::toJSON(os__, "mode", mode, nindent__);
    }
  };
}

#endif
