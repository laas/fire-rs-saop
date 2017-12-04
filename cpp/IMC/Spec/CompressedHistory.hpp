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

#ifndef IMC_COMPRESSEDHISTORY_HPP_INCLUDED_
#define IMC_COMPRESSEDHISTORY_HPP_INCLUDED_

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
  //! Compressed Historic Data Series.
  class CompressedHistory: public Message
  {
  public:
    //! Base Latitude.
    float base_lat;
    //! Base Longitude.
    float base_lon;
    //! Base Timestamp.
    float base_time;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 185;
    }

    static CompressedHistory*
    cast(Message* msg__)
    {
      return (CompressedHistory*)msg__;
    }

    CompressedHistory(void)
    {
      m_header.mgid = CompressedHistory::getIdStatic();
      clear();
    }

    CompressedHistory*
    clone(void) const
    {
      return new CompressedHistory(*this);
    }

    void
    clear(void)
    {
      base_lat = 0;
      base_lon = 0;
      base_time = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::CompressedHistory& other__ = static_cast<const CompressedHistory&>(msg__);
      if (base_lat != other__.base_lat) return false;
      if (base_lon != other__.base_lon) return false;
      if (base_time != other__.base_time) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(base_lat, ptr__);
      ptr__ += IMC::serialize(base_lon, ptr__);
      ptr__ += IMC::serialize(base_time, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(base_lat, bfr__, size__);
      bfr__ += IMC::deserialize(base_lon, bfr__, size__);
      bfr__ += IMC::deserialize(base_time, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(base_lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(base_lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(base_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return CompressedHistory::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "CompressedHistory";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "base_lat", base_lat, nindent__);
      IMC::toJSON(os__, "base_lon", base_lon, nindent__);
      IMC::toJSON(os__, "base_time", base_time, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
