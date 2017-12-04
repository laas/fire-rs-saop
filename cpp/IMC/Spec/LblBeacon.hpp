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

#ifndef IMC_LBLBEACON_HPP_INCLUDED_
#define IMC_LBLBEACON_HPP_INCLUDED_

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
  //! LBL Beacon Configuration.
  class LblBeacon: public Message
  {
  public:
    //! Beacon Name.
    std::string beacon;
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Depth.
    float depth;
    //! Interrogation channel.
    uint8_t query_channel;
    //! Reply channel.
    uint8_t reply_channel;
    //! Transponder delay.
    uint8_t transponder_delay;

    static uint16_t
    getIdStatic(void)
    {
      return 202;
    }

    static LblBeacon*
    cast(Message* msg__)
    {
      return (LblBeacon*)msg__;
    }

    LblBeacon(void)
    {
      m_header.mgid = LblBeacon::getIdStatic();
      clear();
    }

    LblBeacon*
    clone(void) const
    {
      return new LblBeacon(*this);
    }

    void
    clear(void)
    {
      beacon.clear();
      lat = 0;
      lon = 0;
      depth = 0;
      query_channel = 0;
      reply_channel = 0;
      transponder_delay = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::LblBeacon& other__ = static_cast<const LblBeacon&>(msg__);
      if (beacon != other__.beacon) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (depth != other__.depth) return false;
      if (query_channel != other__.query_channel) return false;
      if (reply_channel != other__.reply_channel) return false;
      if (transponder_delay != other__.transponder_delay) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(beacon, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      ptr__ += IMC::serialize(query_channel, ptr__);
      ptr__ += IMC::serialize(reply_channel, ptr__);
      ptr__ += IMC::serialize(transponder_delay, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(beacon, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(query_channel, bfr__, size__);
      bfr__ += IMC::deserialize(reply_channel, bfr__, size__);
      bfr__ += IMC::deserialize(transponder_delay, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(beacon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(query_channel, bfr__, size__);
      bfr__ += IMC::deserialize(reply_channel, bfr__, size__);
      bfr__ += IMC::deserialize(transponder_delay, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return LblBeacon::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "LblBeacon";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 23;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(beacon);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "beacon", beacon, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
      IMC::toJSON(os__, "query_channel", query_channel, nindent__);
      IMC::toJSON(os__, "reply_channel", reply_channel, nindent__);
      IMC::toJSON(os__, "transponder_delay", transponder_delay, nindent__);
    }
  };
}

#endif
