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

#ifndef IMC_CONTROLLOOPS_HPP_INCLUDED_
#define IMC_CONTROLLOOPS_HPP_INCLUDED_

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
  //! Control Loops.
  class ControlLoops: public Message
  {
  public:
    //! Enable.
    enum EnableEnum
    {
      //! Disable.
      CL_DISABLE = 0,
      //! Enable.
      CL_ENABLE = 1
    };

    //! Enable.
    uint8_t enable;
    //! Control Loop Mask.
    uint32_t mask;
    //! Scope Time Reference.
    uint32_t scope_ref;

    static uint16_t
    getIdStatic(void)
    {
      return 507;
    }

    static ControlLoops*
    cast(Message* msg__)
    {
      return (ControlLoops*)msg__;
    }

    ControlLoops(void)
    {
      m_header.mgid = ControlLoops::getIdStatic();
      clear();
    }

    ControlLoops*
    clone(void) const
    {
      return new ControlLoops(*this);
    }

    void
    clear(void)
    {
      enable = 0;
      mask = 0;
      scope_ref = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ControlLoops& other__ = static_cast<const ControlLoops&>(msg__);
      if (enable != other__.enable) return false;
      if (mask != other__.mask) return false;
      if (scope_ref != other__.scope_ref) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(enable, ptr__);
      ptr__ += IMC::serialize(mask, ptr__);
      ptr__ += IMC::serialize(scope_ref, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(enable, bfr__, size__);
      bfr__ += IMC::deserialize(mask, bfr__, size__);
      bfr__ += IMC::deserialize(scope_ref, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(enable, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(mask, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(scope_ref, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ControlLoops::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ControlLoops";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "enable", enable, nindent__);
      IMC::toJSON(os__, "mask", mask, nindent__);
      IMC::toJSON(os__, "scope_ref", scope_ref, nindent__);
    }
  };
}

#endif
