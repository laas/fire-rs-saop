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

#ifndef IMC_USBLFIX_HPP_INCLUDED_
#define IMC_USBLFIX_HPP_INCLUDED_

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
  //! USBL Fix.
  class UsblFix: public Message
  {
  public:
    //! Target.
    uint16_t target;
    //! Latitude (WGS-84).
    double lat;
    //! Longitude (WGS-84).
    double lon;
    //! Z Units.
    uint8_t z_units;
    //! Z Reference.
    float z;

    static uint16_t
    getIdStatic(void)
    {
      return 892;
    }

    static UsblFix*
    cast(Message* msg__)
    {
      return (UsblFix*)msg__;
    }

    UsblFix(void)
    {
      m_header.mgid = UsblFix::getIdStatic();
      clear();
    }

    UsblFix*
    clone(void) const
    {
      return new UsblFix(*this);
    }

    void
    clear(void)
    {
      target = 0;
      lat = 0;
      lon = 0;
      z_units = 0;
      z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblFix& other__ = static_cast<const UsblFix&>(msg__);
      if (target != other__.target) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (z_units != other__.z_units) return false;
      if (z != other__.z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(target, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(target, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(target, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblFix::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblFix";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 23;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "target", target, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
    }
  };
}

#endif
