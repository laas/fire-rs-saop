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

#ifndef IMC_POLYGONVERTEX_HPP_INCLUDED_
#define IMC_POLYGONVERTEX_HPP_INCLUDED_

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
  //! Polygon Vertex.
  class PolygonVertex: public Message
  {
  public:
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;

    static uint16_t
    getIdStatic(void)
    {
      return 474;
    }

    static PolygonVertex*
    cast(Message* msg__)
    {
      return (PolygonVertex*)msg__;
    }

    PolygonVertex(void)
    {
      m_header.mgid = PolygonVertex::getIdStatic();
      clear();
    }

    PolygonVertex*
    clone(void) const
    {
      return new PolygonVertex(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::PolygonVertex& other__ = static_cast<const PolygonVertex&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return PolygonVertex::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "PolygonVertex";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 16;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
    }
  };
}

#endif
