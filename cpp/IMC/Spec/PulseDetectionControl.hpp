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

#ifndef IMC_PULSEDETECTIONCONTROL_HPP_INCLUDED_
#define IMC_PULSEDETECTIONCONTROL_HPP_INCLUDED_

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
  //! Pulse Detection Control.
  class PulseDetectionControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Pulse Detection OFF.
      POP_OFF = 0,
      //! Pulse Detection ON.
      POP_ON = 1
    };

    //! Operation.
    uint8_t op;

    static uint16_t
    getIdStatic(void)
    {
      return 278;
    }

    static PulseDetectionControl*
    cast(Message* msg__)
    {
      return (PulseDetectionControl*)msg__;
    }

    PulseDetectionControl(void)
    {
      m_header.mgid = PulseDetectionControl::getIdStatic();
      clear();
    }

    PulseDetectionControl*
    clone(void) const
    {
      return new PulseDetectionControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PulseDetectionControl& other__ = static_cast<const PulseDetectionControl&>(msg__);
      if (op != other__.op) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PulseDetectionControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PulseDetectionControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
    }
  };
}

#endif
