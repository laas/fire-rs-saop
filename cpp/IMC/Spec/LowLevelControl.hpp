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

#ifndef IMC_LOWLEVELCONTROL_HPP_INCLUDED_
#define IMC_LOWLEVELCONTROL_HPP_INCLUDED_

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
#include "../Spec/ControlCommand.hpp"
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Low Level Control Maneuver.
  class LowLevelControl: public Maneuver
  {
  public:
    //! Control.
    InlineMessage<ControlCommand> control;
    //! Duration.
    uint16_t duration;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 455;
    }

    static LowLevelControl*
    cast(Message* msg__)
    {
      return (LowLevelControl*)msg__;
    }

    LowLevelControl(void)
    {
      m_header.mgid = LowLevelControl::getIdStatic();
      clear();
      control.setParent(this);
    }

    LowLevelControl*
    clone(void) const
    {
      return new LowLevelControl(*this);
    }

    void
    clear(void)
    {
      control.clear();
      duration = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LowLevelControl& other__ = static_cast<const LowLevelControl&>(msg__);
      if (control != other__.control) return false;
      if (duration != other__.duration) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += control.serialize(ptr__);
      ptr__ += IMC::serialize(duration, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += control.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(duration, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += control.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(duration, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LowLevelControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LowLevelControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return control.getSerializationSize() + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      control.toJSON(os__, "control", nindent__);
      IMC::toJSON(os__, "duration", duration, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!control.isNull())
      {
        control.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!control.isNull())
      {
        control.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!control.isNull())
      {
        control.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!control.isNull())
      {
        control.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!control.isNull())
      {
        control.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
