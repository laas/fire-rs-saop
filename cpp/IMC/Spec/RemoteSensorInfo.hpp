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

#ifndef IMC_REMOTESENSORINFO_HPP_INCLUDED_
#define IMC_REMOTESENSORINFO_HPP_INCLUDED_

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
  //! Remote Sensor Info.
  class RemoteSensorInfo: public Message
  {
  public:
    //! Id.
    std::string id;
    //! Class.
    std::string sensor_class;
    //! Latitude.
    double lat;
    //! Longitude.
    double lon;
    //! Altitude.
    float alt;
    //! Heading.
    float heading;
    //! Custom Data.
    std::string data;

    static uint16_t
    getIdStatic(void)
    {
      return 601;
    }

    static RemoteSensorInfo*
    cast(Message* msg__)
    {
      return (RemoteSensorInfo*)msg__;
    }

    RemoteSensorInfo(void)
    {
      m_header.mgid = RemoteSensorInfo::getIdStatic();
      clear();
    }

    RemoteSensorInfo*
    clone(void) const
    {
      return new RemoteSensorInfo(*this);
    }

    void
    clear(void)
    {
      id.clear();
      sensor_class.clear();
      lat = 0;
      lon = 0;
      alt = 0;
      heading = 0;
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RemoteSensorInfo& other__ = static_cast<const RemoteSensorInfo&>(msg__);
      if (id != other__.id) return false;
      if (sensor_class != other__.sensor_class) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (alt != other__.alt) return false;
      if (heading != other__.heading) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(id, ptr__);
      ptr__ += IMC::serialize(sensor_class, ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(alt, ptr__);
      ptr__ += IMC::serialize(heading, ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(id, bfr__, size__);
      bfr__ += IMC::deserialize(sensor_class, bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(alt, bfr__, size__);
      bfr__ += IMC::deserialize(heading, bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sensor_class, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(alt, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(heading, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RemoteSensorInfo::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RemoteSensorInfo";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 24;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(id) + IMC::getSerializationSize(sensor_class) + IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "id", id, nindent__);
      IMC::toJSON(os__, "sensor_class", sensor_class, nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "alt", alt, nindent__);
      IMC::toJSON(os__, "heading", heading, nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }
  };
}

#endif
