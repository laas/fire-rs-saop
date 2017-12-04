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

#ifndef IMC_IRIDIUMMSGRX_HPP_INCLUDED_
#define IMC_IRIDIUMMSGRX_HPP_INCLUDED_

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
  //! Received Iridium Message.
  class IridiumMsgRx: public Message
  {
  public:
    //! Origin Identifier.
    std::string origin;
    //! Timestamp.
    double htime;
    //! Latitude Reference.
    double lat;
    //! Longitude Reference.
    double lon;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 170;
    }

    static IridiumMsgRx*
    cast(Message* msg__)
    {
      return (IridiumMsgRx*)msg__;
    }

    IridiumMsgRx(void)
    {
      m_header.mgid = IridiumMsgRx::getIdStatic();
      clear();
    }

    IridiumMsgRx*
    clone(void) const
    {
      return new IridiumMsgRx(*this);
    }

    void
    clear(void)
    {
      origin.clear();
      htime = 0;
      lat = 0;
      lon = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::IridiumMsgRx& other__ = static_cast<const IridiumMsgRx&>(msg__);
      if (origin != other__.origin) return false;
      if (htime != other__.htime) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(origin, ptr__);
      ptr__ += IMC::serialize(htime, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(origin, bfr__, size__);
      bfr__ += IMC::deserialize(htime, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(origin, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(htime, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return IridiumMsgRx::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "IridiumMsgRx";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 24;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(origin) + IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "origin", origin, nindent__);
      IMC::toJSON(os__, "htime", htime, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
