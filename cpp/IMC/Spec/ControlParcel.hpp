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

#ifndef IMC_CONTROLPARCEL_HPP_INCLUDED_
#define IMC_CONTROLPARCEL_HPP_INCLUDED_

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
  //! Control Parcel.
  class ControlParcel: public Message
  {
  public:
    //! Proportional Parcel.
    float p;
    //! Integrative Parcel.
    float i;
    //! Derivative Parcel.
    float d;
    //! Anti-Windup Parcel.
    float a;

    static uint16_t
    getIdStatic(void)
    {
      return 412;
    }

    static ControlParcel*
    cast(Message* msg__)
    {
      return (ControlParcel*)msg__;
    }

    ControlParcel(void)
    {
      m_header.mgid = ControlParcel::getIdStatic();
      clear();
    }

    ControlParcel*
    clone(void) const
    {
      return new ControlParcel(*this);
    }

    void
    clear(void)
    {
      p = 0;
      i = 0;
      d = 0;
      a = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ControlParcel& other__ = static_cast<const ControlParcel&>(msg__);
      if (p != other__.p) return false;
      if (i != other__.i) return false;
      if (d != other__.d) return false;
      if (a != other__.a) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(p, ptr__);
      ptr__ += IMC::serialize(i, ptr__);
      ptr__ += IMC::serialize(d, ptr__);
      ptr__ += IMC::serialize(a, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(p, bfr__, size__);
      bfr__ += IMC::deserialize(i, bfr__, size__);
      bfr__ += IMC::deserialize(d, bfr__, size__);
      bfr__ += IMC::deserialize(a, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(p, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(i, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(d, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(a, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ControlParcel::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ControlParcel";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 16;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "p", p, nindent__);
      IMC::toJSON(os__, "i", i, nindent__);
      IMC::toJSON(os__, "d", d, nindent__);
      IMC::toJSON(os__, "a", a, nindent__);
    }
  };
}

#endif
