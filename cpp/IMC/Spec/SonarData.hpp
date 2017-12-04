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

#ifndef IMC_SONARDATA_HPP_INCLUDED_
#define IMC_SONARDATA_HPP_INCLUDED_

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
#include "../Spec/BeamConfig.hpp"

namespace IMC
{
  //! Sonar Data.
  class SonarData: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Sidescan.
      ST_SIDESCAN = 0,
      //! Echo Sounder.
      ST_ECHOSOUNDER = 1,
      //! Multibeam.
      ST_MULTIBEAM = 2
    };

    //! Type.
    uint8_t type;
    //! Frequency.
    uint32_t frequency;
    //! Minimum Range.
    uint16_t min_range;
    //! Maximum Range.
    uint16_t max_range;
    //! Bits Per Data Point.
    uint8_t bits_per_point;
    //! Scaling Factor.
    float scale_factor;
    //! Beam Configuration.
    MessageList<BeamConfig> beam_config;
    //! Data.
    std::vector<char> data;

    static uint16_t
    getIdStatic(void)
    {
      return 276;
    }

    static SonarData*
    cast(Message* msg__)
    {
      return (SonarData*)msg__;
    }

    SonarData(void)
    {
      m_header.mgid = SonarData::getIdStatic();
      clear();
      beam_config.setParent(this);
    }

    SonarData*
    clone(void) const
    {
      return new SonarData(*this);
    }

    void
    clear(void)
    {
      type = 0;
      frequency = 0;
      min_range = 0;
      max_range = 0;
      bits_per_point = 0;
      scale_factor = 0;
      beam_config.clear();
      data.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SonarData& other__ = static_cast<const SonarData&>(msg__);
      if (type != other__.type) return false;
      if (frequency != other__.frequency) return false;
      if (min_range != other__.min_range) return false;
      if (max_range != other__.max_range) return false;
      if (bits_per_point != other__.bits_per_point) return false;
      if (scale_factor != other__.scale_factor) return false;
      if (beam_config != other__.beam_config) return false;
      if (data != other__.data) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(frequency, ptr__);
      ptr__ += IMC::serialize(min_range, ptr__);
      ptr__ += IMC::serialize(max_range, ptr__);
      ptr__ += IMC::serialize(bits_per_point, ptr__);
      ptr__ += IMC::serialize(scale_factor, ptr__);
      ptr__ += beam_config.serialize(ptr__);
      ptr__ += IMC::serialize(data, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(frequency, bfr__, size__);
      bfr__ += IMC::deserialize(min_range, bfr__, size__);
      bfr__ += IMC::deserialize(max_range, bfr__, size__);
      bfr__ += IMC::deserialize(bits_per_point, bfr__, size__);
      bfr__ += IMC::deserialize(scale_factor, bfr__, size__);
      bfr__ += beam_config.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(frequency, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(min_range, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max_range, bfr__, size__);
      bfr__ += IMC::deserialize(bits_per_point, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(scale_factor, bfr__, size__);
      bfr__ += beam_config.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(data, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SonarData::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SonarData";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 14;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return beam_config.getSerializationSize() + IMC::getSerializationSize(data);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "frequency", frequency, nindent__);
      IMC::toJSON(os__, "min_range", min_range, nindent__);
      IMC::toJSON(os__, "max_range", max_range, nindent__);
      IMC::toJSON(os__, "bits_per_point", bits_per_point, nindent__);
      IMC::toJSON(os__, "scale_factor", scale_factor, nindent__);
      beam_config.toJSON(os__, "beam_config", nindent__);
      IMC::toJSON(os__, "data", data, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      beam_config.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      beam_config.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      beam_config.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      beam_config.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      beam_config.setDestinationEntity(value__);
    }
  };
}

#endif
