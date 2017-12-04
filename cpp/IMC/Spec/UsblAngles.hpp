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

#ifndef IMC_USBLANGLES_HPP_INCLUDED_
#define IMC_USBLANGLES_HPP_INCLUDED_

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
  //! USBL Angles.
  class UsblAngles: public Message
  {
  public:
    //! Target.
    uint16_t target;
    //! Bearing.
    float bearing;
    //! Elevation.
    float elevation;

    static uint16_t
    getIdStatic(void)
    {
      return 890;
    }

    static UsblAngles*
    cast(Message* msg__)
    {
      return (UsblAngles*)msg__;
    }

    UsblAngles(void)
    {
      m_header.mgid = UsblAngles::getIdStatic();
      clear();
    }

    UsblAngles*
    clone(void) const
    {
      return new UsblAngles(*this);
    }

    void
    clear(void)
    {
      target = 0;
      bearing = 0;
      elevation = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblAngles& other__ = static_cast<const UsblAngles&>(msg__);
      if (target != other__.target) return false;
      if (bearing != other__.bearing) return false;
      if (elevation != other__.elevation) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(target, ptr__);
      ptr__ += IMC::serialize(bearing, ptr__);
      ptr__ += IMC::serialize(elevation, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(target, bfr__, size__);
      bfr__ += IMC::deserialize(bearing, bfr__, size__);
      bfr__ += IMC::deserialize(elevation, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(target, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bearing, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(elevation, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblAngles::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblAngles";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 10;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "target", target, nindent__);
      IMC::toJSON(os__, "bearing", bearing, nindent__);
      IMC::toJSON(os__, "elevation", elevation, nindent__);
    }
  };
}

#endif
