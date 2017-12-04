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

#ifndef IMC_REGISTERMANEUVER_HPP_INCLUDED_
#define IMC_REGISTERMANEUVER_HPP_INCLUDED_

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
  //! Register Maneuver.
  class RegisterManeuver: public Message
  {
  public:
    //! Maneuver ID.
    uint16_t mid;

    static uint16_t
    getIdStatic(void)
    {
      return 469;
    }

    static RegisterManeuver*
    cast(Message* msg__)
    {
      return (RegisterManeuver*)msg__;
    }

    RegisterManeuver(void)
    {
      m_header.mgid = RegisterManeuver::getIdStatic();
      clear();
    }

    RegisterManeuver*
    clone(void) const
    {
      return new RegisterManeuver(*this);
    }

    void
    clear(void)
    {
      mid = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RegisterManeuver& other__ = static_cast<const RegisterManeuver&>(msg__);
      if (mid != other__.mid) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(mid, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(mid, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(mid, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RegisterManeuver::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RegisterManeuver";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 2;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "mid", mid, nindent__);
    }
  };
}

#endif
