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

#ifndef IMC_TACHOGRAPH_HPP_INCLUDED_
#define IMC_TACHOGRAPH_HPP_INCLUDED_

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
  //! Tachograph.
  class Tachograph: public Message
  {
  public:
    //! Last Service Timestamp.
    double timestamp_last_service;
    //! Time - Next Service.
    float time_next_service;
    //! Time Motor - Next Service.
    float time_motor_next_service;
    //! Time Idle - Ground.
    float time_idle_ground;
    //! Time Idle - Air.
    float time_idle_air;
    //! Time Idle - Water.
    float time_idle_water;
    //! Time Idle - Underwater.
    float time_idle_underwater;
    //! Time Idle - Unknown.
    float time_idle_unknown;
    //! Time Motor - Ground.
    float time_motor_ground;
    //! Time Motor - Air.
    float time_motor_air;
    //! Time Motor - Water.
    float time_motor_water;
    //! Time Motor - Underwater.
    float time_motor_underwater;
    //! Time Motor - Unknown.
    float time_motor_unknown;
    //! Recorded RPMs - Minimum.
    int16_t rpm_min;
    //! Recorded RPMs - Maximum.
    int16_t rpm_max;
    //! Recorded Depth - Maximum.
    float depth_max;

    static uint16_t
    getIdStatic(void)
    {
      return 905;
    }

    static Tachograph*
    cast(Message* msg__)
    {
      return (Tachograph*)msg__;
    }

    Tachograph(void)
    {
      m_header.mgid = Tachograph::getIdStatic();
      clear();
    }

    Tachograph*
    clone(void) const
    {
      return new Tachograph(*this);
    }

    void
    clear(void)
    {
      timestamp_last_service = 0;
      time_next_service = 0;
      time_motor_next_service = 0;
      time_idle_ground = 0;
      time_idle_air = 0;
      time_idle_water = 0;
      time_idle_underwater = 0;
      time_idle_unknown = 0;
      time_motor_ground = 0;
      time_motor_air = 0;
      time_motor_water = 0;
      time_motor_underwater = 0;
      time_motor_unknown = 0;
      rpm_min = 0;
      rpm_max = 0;
      depth_max = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Tachograph& other__ = static_cast<const Tachograph&>(msg__);
      if (timestamp_last_service != other__.timestamp_last_service) return false;
      if (time_next_service != other__.time_next_service) return false;
      if (time_motor_next_service != other__.time_motor_next_service) return false;
      if (time_idle_ground != other__.time_idle_ground) return false;
      if (time_idle_air != other__.time_idle_air) return false;
      if (time_idle_water != other__.time_idle_water) return false;
      if (time_idle_underwater != other__.time_idle_underwater) return false;
      if (time_idle_unknown != other__.time_idle_unknown) return false;
      if (time_motor_ground != other__.time_motor_ground) return false;
      if (time_motor_air != other__.time_motor_air) return false;
      if (time_motor_water != other__.time_motor_water) return false;
      if (time_motor_underwater != other__.time_motor_underwater) return false;
      if (time_motor_unknown != other__.time_motor_unknown) return false;
      if (rpm_min != other__.rpm_min) return false;
      if (rpm_max != other__.rpm_max) return false;
      if (depth_max != other__.depth_max) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(timestamp_last_service, ptr__);
      ptr__ += IMC::serialize(time_next_service, ptr__);
      ptr__ += IMC::serialize(time_motor_next_service, ptr__);
      ptr__ += IMC::serialize(time_idle_ground, ptr__);
      ptr__ += IMC::serialize(time_idle_air, ptr__);
      ptr__ += IMC::serialize(time_idle_water, ptr__);
      ptr__ += IMC::serialize(time_idle_underwater, ptr__);
      ptr__ += IMC::serialize(time_idle_unknown, ptr__);
      ptr__ += IMC::serialize(time_motor_ground, ptr__);
      ptr__ += IMC::serialize(time_motor_air, ptr__);
      ptr__ += IMC::serialize(time_motor_water, ptr__);
      ptr__ += IMC::serialize(time_motor_underwater, ptr__);
      ptr__ += IMC::serialize(time_motor_unknown, ptr__);
      ptr__ += IMC::serialize(rpm_min, ptr__);
      ptr__ += IMC::serialize(rpm_max, ptr__);
      ptr__ += IMC::serialize(depth_max, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(timestamp_last_service, bfr__, size__);
      bfr__ += IMC::deserialize(time_next_service, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_next_service, bfr__, size__);
      bfr__ += IMC::deserialize(time_idle_ground, bfr__, size__);
      bfr__ += IMC::deserialize(time_idle_air, bfr__, size__);
      bfr__ += IMC::deserialize(time_idle_water, bfr__, size__);
      bfr__ += IMC::deserialize(time_idle_underwater, bfr__, size__);
      bfr__ += IMC::deserialize(time_idle_unknown, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_ground, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_air, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_water, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_underwater, bfr__, size__);
      bfr__ += IMC::deserialize(time_motor_unknown, bfr__, size__);
      bfr__ += IMC::deserialize(rpm_min, bfr__, size__);
      bfr__ += IMC::deserialize(rpm_max, bfr__, size__);
      bfr__ += IMC::deserialize(depth_max, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(timestamp_last_service, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_next_service, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_next_service, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_idle_ground, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_idle_air, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_idle_water, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_idle_underwater, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_idle_unknown, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_ground, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_air, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_water, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_underwater, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time_motor_unknown, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth_max, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Tachograph::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Tachograph";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 64;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "timestamp_last_service", timestamp_last_service, nindent__);
      IMC::toJSON(os__, "time_next_service", time_next_service, nindent__);
      IMC::toJSON(os__, "time_motor_next_service", time_motor_next_service, nindent__);
      IMC::toJSON(os__, "time_idle_ground", time_idle_ground, nindent__);
      IMC::toJSON(os__, "time_idle_air", time_idle_air, nindent__);
      IMC::toJSON(os__, "time_idle_water", time_idle_water, nindent__);
      IMC::toJSON(os__, "time_idle_underwater", time_idle_underwater, nindent__);
      IMC::toJSON(os__, "time_idle_unknown", time_idle_unknown, nindent__);
      IMC::toJSON(os__, "time_motor_ground", time_motor_ground, nindent__);
      IMC::toJSON(os__, "time_motor_air", time_motor_air, nindent__);
      IMC::toJSON(os__, "time_motor_water", time_motor_water, nindent__);
      IMC::toJSON(os__, "time_motor_underwater", time_motor_underwater, nindent__);
      IMC::toJSON(os__, "time_motor_unknown", time_motor_unknown, nindent__);
      IMC::toJSON(os__, "rpm_min", rpm_min, nindent__);
      IMC::toJSON(os__, "rpm_max", rpm_max, nindent__);
      IMC::toJSON(os__, "depth_max", depth_max, nindent__);
    }
  };
}

#endif
