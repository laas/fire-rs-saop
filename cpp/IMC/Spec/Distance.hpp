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

#ifndef IMC_DISTANCE_HPP_INCLUDED_
#define IMC_DISTANCE_HPP_INCLUDED_

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
#include "../Spec/DeviceState.hpp"
#include "../Spec/BeamConfig.hpp"

namespace IMC
{
  //! Distance.
  class Distance: public Message
  {
  public:
    //! Validity.
    enum ValidityEnum
    {
      //! Invalid.
      DV_INVALID = 0,
      //! Valid.
      DV_VALID = 1
    };

    //! Validity.
    uint8_t validity;
    //! Location.
    MessageList<DeviceState> location;
    //! Beam Configuration.
    MessageList<BeamConfig> beam_config;
    //! Measured Distance.
    float value;

    static uint16_t
    getIdStatic(void)
    {
      return 262;
    }

    static Distance*
    cast(Message* msg__)
    {
      return (Distance*)msg__;
    }

    Distance(void)
    {
      m_header.mgid = Distance::getIdStatic();
      clear();
      location.setParent(this);
      beam_config.setParent(this);
    }

    Distance*
    clone(void) const
    {
      return new Distance(*this);
    }

    void
    clear(void)
    {
      validity = 0;
      location.clear();
      beam_config.clear();
      value = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Distance& other__ = static_cast<const Distance&>(msg__);
      if (validity != other__.validity) return false;
      if (location != other__.location) return false;
      if (beam_config != other__.beam_config) return false;
      if (value != other__.value) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(validity, ptr__);
      ptr__ += location.serialize(ptr__);
      ptr__ += beam_config.serialize(ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += location.deserialize(bfr__, size__);
      bfr__ += beam_config.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(validity, bfr__, size__);
      bfr__ += location.reverseDeserialize(bfr__, size__);
      bfr__ += beam_config.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(value, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Distance::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Distance";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 5;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return location.getSerializationSize() + beam_config.getSerializationSize();
    }

    double
    getValueFP(void) const
    {
      return static_cast<double>(value);
    }

    void
    setValueFP(double val)
    {
      value = static_cast<float>(val);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "validity", validity, nindent__);
      location.toJSON(os__, "location", nindent__);
      beam_config.toJSON(os__, "beam_config", nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      location.setTimeStamp(value__);

      beam_config.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      location.setSource(value__);

      beam_config.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      location.setSourceEntity(value__);

      beam_config.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      location.setDestination(value__);

      beam_config.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      location.setDestinationEntity(value__);

      beam_config.setDestinationEntity(value__);
    }
  };
}

#endif
