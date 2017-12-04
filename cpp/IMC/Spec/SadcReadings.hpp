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

#ifndef IMC_SADCREADINGS_HPP_INCLUDED_
#define IMC_SADCREADINGS_HPP_INCLUDED_

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
  //! SADC Readings.
  class SadcReadings: public Message
  {
  public:
    //! Gain.
    enum GainEnum
    {
      //! x1.
      GAIN_X1 = 0,
      //! x10.
      GAIN_X10 = 1,
      //! x100.
      GAIN_X100 = 2
    };

    //! Channel.
    int8_t channel;
    //! Value.
    int32_t value;
    //! Gain.
    uint8_t gain;

    static uint16_t
    getIdStatic(void)
    {
      return 907;
    }

    static SadcReadings*
    cast(Message* msg__)
    {
      return (SadcReadings*)msg__;
    }

    SadcReadings(void)
    {
      m_header.mgid = SadcReadings::getIdStatic();
      clear();
    }

    SadcReadings*
    clone(void) const
    {
      return new SadcReadings(*this);
    }

    void
    clear(void)
    {
      channel = 0;
      value = 0;
      gain = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SadcReadings& other__ = static_cast<const SadcReadings&>(msg__);
      if (channel != other__.channel) return false;
      if (value != other__.value) return false;
      if (gain != other__.gain) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(channel, ptr__);
      ptr__ += IMC::serialize(value, ptr__);
      ptr__ += IMC::serialize(gain, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(channel, bfr__, size__);
      bfr__ += IMC::deserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(gain, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(channel, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(value, bfr__, size__);
      bfr__ += IMC::deserialize(gain, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SadcReadings::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SadcReadings";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    double
    getValueFP(void) const
    {
      return static_cast<double>(value);
    }

    void
    setValueFP(double val)
    {
      value = static_cast<int32_t>(val);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "channel", channel, nindent__);
      IMC::toJSON(os__, "value", value, nindent__);
      IMC::toJSON(os__, "gain", gain, nindent__);
    }
  };
}

#endif
