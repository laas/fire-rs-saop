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

#ifndef IMC_HISTORICSONARDATA_HPP_INCLUDED_
#define IMC_HISTORICSONARDATA_HPP_INCLUDED_

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
  //! Historic Sonar Data.
  class HistoricSonarData: public Message
  {
  public:
    //! Encoding.
    enum EncodingEnum
    {
      //! One Byte Per Pixel.
      ENC_ONE_BYTE_PER_PIXEL = 0,
      //! PNG compressed image.
      ENC_PNG = 1,
      //! JPEG compressed image.
      ENC_JPEG = 2
    };

    //! Altitude.
    float altitude;
    //! Width.
    float width;
    //! Length.
    float length;
    //! Bearing.
    float bearing;
    //! Pixels Per Line.
    int16_t pxl;
    //! Encoding.
    uint8_t encoding;
    //! SonarData.
    std::vector<char> sonar_data;

    static uint16_t
    getIdStatic(void)
    {
      return 109;
    }

    static HistoricSonarData*
    cast(Message* msg__)
    {
      return (HistoricSonarData*)msg__;
    }

    HistoricSonarData(void)
    {
      m_header.mgid = HistoricSonarData::getIdStatic();
      clear();
    }

    HistoricSonarData*
    clone(void) const
    {
      return new HistoricSonarData(*this);
    }

    void
    clear(void)
    {
      altitude = 0;
      width = 0;
      length = 0;
      bearing = 0;
      pxl = 0;
      encoding = 0;
      sonar_data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::HistoricSonarData& other__ = static_cast<const HistoricSonarData&>(msg__);
      if (altitude != other__.altitude) return false;
      if (width != other__.width) return false;
      if (length != other__.length) return false;
      if (bearing != other__.bearing) return false;
      if (pxl != other__.pxl) return false;
      if (encoding != other__.encoding) return false;
      if (sonar_data != other__.sonar_data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(altitude, ptr__);
      ptr__ += IMC::serialize(width, ptr__);
      ptr__ += IMC::serialize(length, ptr__);
      ptr__ += IMC::serialize(bearing, ptr__);
      ptr__ += IMC::serialize(pxl, ptr__);
      ptr__ += IMC::serialize(encoding, ptr__);
      ptr__ += IMC::serialize(sonar_data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(altitude, bfr__, size__);
      bfr__ += IMC::deserialize(width, bfr__, size__);
      bfr__ += IMC::deserialize(length, bfr__, size__);
      bfr__ += IMC::deserialize(bearing, bfr__, size__);
      bfr__ += IMC::deserialize(pxl, bfr__, size__);
      bfr__ += IMC::deserialize(encoding, bfr__, size__);
      bfr__ += IMC::deserialize(sonar_data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(altitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(width, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(length, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bearing, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pxl, bfr__, size__);
      bfr__ += IMC::deserialize(encoding, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sonar_data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return HistoricSonarData::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "HistoricSonarData";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 19;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(sonar_data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "altitude", altitude, nindent__);
      IMC::toJSON(os__, "width", width, nindent__);
      IMC::toJSON(os__, "length", length, nindent__);
      IMC::toJSON(os__, "bearing", bearing, nindent__);
      IMC::toJSON(os__, "pxl", pxl, nindent__);
      IMC::toJSON(os__, "encoding", encoding, nindent__);
      IMC::toJSON(os__, "sonar_data", sonar_data, nindent__);
    }
  };
}

#endif
