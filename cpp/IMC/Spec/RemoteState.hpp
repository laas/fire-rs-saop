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

#ifndef IMC_REMOTESTATE_HPP_INCLUDED_
#define IMC_REMOTESTATE_HPP_INCLUDED_

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
  //! Remote State.
  class RemoteState: public Message
  {
  public:
    //! Latitude WGS-84.
    float lat;
    //! Longitude WGS-84.
    float lon;
    //! Depth.
    uint8_t depth;
    //! Speed.
    float speed;
    //! Heading.
    float psi;

    static uint16_t
    getIdStatic(void)
    {
      return 750;
    }

    static RemoteState*
    cast(Message* msg__)
    {
      return (RemoteState*)msg__;
    }

    RemoteState(void)
    {
      m_header.mgid = RemoteState::getIdStatic();
      clear();
    }

    RemoteState*
    clone(void) const
    {
      return new RemoteState(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      depth = 0;
      speed = 0;
      psi = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RemoteState& other__ = static_cast<const RemoteState&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (depth != other__.depth) return false;
      if (speed != other__.speed) return false;
      if (psi != other__.psi) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(psi, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(psi, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(psi, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RemoteState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RemoteState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 17;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "psi", psi, nindent__);
    }
  };
}

#endif
