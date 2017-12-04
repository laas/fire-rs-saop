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

#ifndef IMC_HISTORICTELEMETRY_HPP_INCLUDED_
#define IMC_HISTORICTELEMETRY_HPP_INCLUDED_

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
  //! Historic Telemetry.
  class HistoricTelemetry: public Message
  {
  public:
    //! Altitude.
    float altitude;
    //! Roll.
    uint16_t roll;
    //! Pitch.
    uint16_t pitch;
    //! Yaw.
    uint16_t yaw;
    //! Speed.
    int16_t speed;

    static uint16_t
    getIdStatic(void)
    {
      return 108;
    }

    static HistoricTelemetry*
    cast(Message* msg__)
    {
      return (HistoricTelemetry*)msg__;
    }

    HistoricTelemetry(void)
    {
      m_header.mgid = HistoricTelemetry::getIdStatic();
      clear();
    }

    HistoricTelemetry*
    clone(void) const
    {
      return new HistoricTelemetry(*this);
    }

    void
    clear(void)
    {
      altitude = 0;
      roll = 0;
      pitch = 0;
      yaw = 0;
      speed = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::HistoricTelemetry& other__ = static_cast<const HistoricTelemetry&>(msg__);
      if (altitude != other__.altitude) return false;
      if (roll != other__.roll) return false;
      if (pitch != other__.pitch) return false;
      if (yaw != other__.yaw) return false;
      if (speed != other__.speed) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(altitude, ptr__);
      ptr__ += IMC::serialize(roll, ptr__);
      ptr__ += IMC::serialize(pitch, ptr__);
      ptr__ += IMC::serialize(yaw, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(altitude, bfr__, size__);
      bfr__ += IMC::deserialize(roll, bfr__, size__);
      bfr__ += IMC::deserialize(pitch, bfr__, size__);
      bfr__ += IMC::deserialize(yaw, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(altitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(roll, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pitch, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(yaw, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return HistoricTelemetry::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "HistoricTelemetry";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "altitude", altitude, nindent__);
      IMC::toJSON(os__, "roll", roll, nindent__);
      IMC::toJSON(os__, "pitch", pitch, nindent__);
      IMC::toJSON(os__, "yaw", yaw, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
    }
  };
}

#endif
