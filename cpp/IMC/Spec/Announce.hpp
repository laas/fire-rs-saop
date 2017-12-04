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

#ifndef IMC_ANNOUNCE_HPP_INCLUDED_
#define IMC_ANNOUNCE_HPP_INCLUDED_

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
  //! Announce.
  class Announce: public Message
  {
  public:
    //! System Name.
    std::string sys_name;
    //! System Type.
    uint8_t sys_type;
    //! Control Owner.
    uint16_t owner;
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Height WGS-84.
    float height;
    //! Services.
    std::string services;

    static uint16_t
    getIdStatic(void)
    {
      return 151;
    }

    static Announce*
    cast(Message* msg__)
    {
      return (Announce*)msg__;
    }

    Announce(void)
    {
      m_header.mgid = Announce::getIdStatic();
      clear();
    }

    Announce*
    clone(void) const
    {
      return new Announce(*this);
    }

    void
    clear(void)
    {
      sys_name.clear();
      sys_type = 0;
      owner = 0;
      lat = 0;
      lon = 0;
      height = 0;
      services.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Announce& other__ = static_cast<const Announce&>(msg__);
      if (sys_name != other__.sys_name) return false;
      if (sys_type != other__.sys_type) return false;
      if (owner != other__.owner) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (height != other__.height) return false;
      if (services != other__.services) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(sys_name, ptr__);
      ptr__ += IMC::serialize(sys_type, ptr__);
      ptr__ += IMC::serialize(owner, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(height, ptr__);
      ptr__ += IMC::serialize(services, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(sys_name, bfr__, size__);
      bfr__ += IMC::deserialize(sys_type, bfr__, size__);
      bfr__ += IMC::deserialize(owner, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(height, bfr__, size__);
      bfr__ += IMC::deserialize(services, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(sys_name, bfr__, size__);
      bfr__ += IMC::deserialize(sys_type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(owner, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(height, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(services, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Announce::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Announce";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 23;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(sys_name) + IMC::getSerializationSize(services);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "sys_name", sys_name, nindent__);
      IMC::toJSON(os__, "sys_type", sys_type, nindent__);
      IMC::toJSON(os__, "owner", owner, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "height", height, nindent__);
      IMC::toJSON(os__, "services", services, nindent__);
    }
  };
}

#endif
