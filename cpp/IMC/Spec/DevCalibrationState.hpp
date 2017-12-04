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

#ifndef IMC_DEVCALIBRATIONSTATE_HPP_INCLUDED_
#define IMC_DEVCALIBRATIONSTATE_HPP_INCLUDED_

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
  //! Device Calibration State.
  class DevCalibrationState: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Previous Step Not Supported.
      DCS_PREVIOUS_NOT_SUPPORTED = 0x01,
      //! Next Step Not Supported.
      DCS_NEXT_NOT_SUPPORTED = 0x02,
      //! Waiting Device Calibration Control.
      DCS_WAITING_CONTROL = 0x04,
      //! Calibration Error.
      DCS_ERROR = 0x08,
      //! Calibration Procedure Completed.
      DCS_COMPLETED = 0x10
    };

    //! Total Steps.
    uint8_t total_steps;
    //! Current Step Number.
    uint8_t step_number;
    //! Description.
    std::string step;
    //! Flags.
    uint8_t flags;

    static uint16_t
    getIdStatic(void)
    {
      return 13;
    }

    static DevCalibrationState*
    cast(Message* msg__)
    {
      return (DevCalibrationState*)msg__;
    }

    DevCalibrationState(void)
    {
      m_header.mgid = DevCalibrationState::getIdStatic();
      clear();
    }

    DevCalibrationState*
    clone(void) const
    {
      return new DevCalibrationState(*this);
    }

    void
    clear(void)
    {
      total_steps = 0;
      step_number = 0;
      step.clear();
      flags = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DevCalibrationState& other__ = static_cast<const DevCalibrationState&>(msg__);
      if (total_steps != other__.total_steps) return false;
      if (step_number != other__.step_number) return false;
      if (step != other__.step) return false;
      if (flags != other__.flags) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(total_steps, ptr__);
      ptr__ += IMC::serialize(step_number, ptr__);
      ptr__ += IMC::serialize(step, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(total_steps, bfr__, size__);
      bfr__ += IMC::deserialize(step_number, bfr__, size__);
      bfr__ += IMC::deserialize(step, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(total_steps, bfr__, size__);
      bfr__ += IMC::deserialize(step_number, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(step, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DevCalibrationState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DevCalibrationState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 3;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(step);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "total_steps", total_steps, nindent__);
      IMC::toJSON(os__, "step_number", step_number, nindent__);
      IMC::toJSON(os__, "step", step, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
    }
  };
}

#endif
