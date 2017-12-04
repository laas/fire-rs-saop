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

#ifndef IMC_VEHICLEOPERATIONALLIMITS_HPP_INCLUDED_
#define IMC_VEHICLEOPERATIONALLIMITS_HPP_INCLUDED_

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
  //! Vehicle Operational Limits.
  class VehicleOperationalLimits: public Message
  {
  public:
    //! Action on the vehicle operational limits.
    enum ActiononthevehicleoperationallimitsEnum
    {
      //! Request.
      OP_REQUEST = 0,
      //! Set.
      OP_SET = 1,
      //! Report.
      OP_REPORT = 2
    };

    //! Action on the vehicle operational limits.
    uint8_t op;
    //! Minimum speed.
    float speed_min;
    //! Maximum speed.
    float speed_max;
    //! Longitudinal maximum acceleration.
    float long_accel;
    //! Maximum MSL altitude.
    float alt_max_msl;
    //! Maximum Dive Rate Speed Fraction.
    float dive_fraction_max;
    //! Maximum Climb Rate Speed Fraction.
    float climb_fraction_max;
    //! Bank limit.
    float bank_max;
    //! Bank rate limit.
    float p_max;
    //! Minimum pitch angle.
    float pitch_min;
    //! Maximum pitch angle.
    float pitch_max;
    //! Maximum pitch rate.
    float q_max;
    //! Minimum load factor.
    float g_min;
    //! Maximum load factor.
    float g_max;
    //! Maximum lateral load factor.
    float g_lat_max;
    //! Minimum RPMs.
    float rpm_min;
    //! Maximum RPMs.
    float rpm_max;
    //! Maximum RPM rate.
    float rpm_rate_max;

    static uint16_t
    getIdStatic(void)
    {
      return 16;
    }

    static VehicleOperationalLimits*
    cast(Message* msg__)
    {
      return (VehicleOperationalLimits*)msg__;
    }

    VehicleOperationalLimits(void)
    {
      m_header.mgid = VehicleOperationalLimits::getIdStatic();
      clear();
    }

    VehicleOperationalLimits*
    clone(void) const
    {
      return new VehicleOperationalLimits(*this);
    }

    void
    clear(void)
    {
      op = 0;
      speed_min = 0;
      speed_max = 0;
      long_accel = 0;
      alt_max_msl = 0;
      dive_fraction_max = 0;
      climb_fraction_max = 0;
      bank_max = 0;
      p_max = 0;
      pitch_min = 0;
      pitch_max = 0;
      q_max = 0;
      g_min = 0;
      g_max = 0;
      g_lat_max = 0;
      rpm_min = 0;
      rpm_max = 0;
      rpm_rate_max = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleOperationalLimits& other__ = static_cast<const VehicleOperationalLimits&>(msg__);
      if (op != other__.op) return false;
      if (speed_min != other__.speed_min) return false;
      if (speed_max != other__.speed_max) return false;
      if (long_accel != other__.long_accel) return false;
      if (alt_max_msl != other__.alt_max_msl) return false;
      if (dive_fraction_max != other__.dive_fraction_max) return false;
      if (climb_fraction_max != other__.climb_fraction_max) return false;
      if (bank_max != other__.bank_max) return false;
      if (p_max != other__.p_max) return false;
      if (pitch_min != other__.pitch_min) return false;
      if (pitch_max != other__.pitch_max) return false;
      if (q_max != other__.q_max) return false;
      if (g_min != other__.g_min) return false;
      if (g_max != other__.g_max) return false;
      if (g_lat_max != other__.g_lat_max) return false;
      if (rpm_min != other__.rpm_min) return false;
      if (rpm_max != other__.rpm_max) return false;
      if (rpm_rate_max != other__.rpm_rate_max) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(speed_min, ptr__);
      ptr__ += IMC::serialize(speed_max, ptr__);
      ptr__ += IMC::serialize(long_accel, ptr__);
      ptr__ += IMC::serialize(alt_max_msl, ptr__);
      ptr__ += IMC::serialize(dive_fraction_max, ptr__);
      ptr__ += IMC::serialize(climb_fraction_max, ptr__);
      ptr__ += IMC::serialize(bank_max, ptr__);
      ptr__ += IMC::serialize(p_max, ptr__);
      ptr__ += IMC::serialize(pitch_min, ptr__);
      ptr__ += IMC::serialize(pitch_max, ptr__);
      ptr__ += IMC::serialize(q_max, ptr__);
      ptr__ += IMC::serialize(g_min, ptr__);
      ptr__ += IMC::serialize(g_max, ptr__);
      ptr__ += IMC::serialize(g_lat_max, ptr__);
      ptr__ += IMC::serialize(rpm_min, ptr__);
      ptr__ += IMC::serialize(rpm_max, ptr__);
      ptr__ += IMC::serialize(rpm_rate_max, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(speed_min, bfr__, size__);
      bfr__ += IMC::deserialize(speed_max, bfr__, size__);
      bfr__ += IMC::deserialize(long_accel, bfr__, size__);
      bfr__ += IMC::deserialize(alt_max_msl, bfr__, size__);
      bfr__ += IMC::deserialize(dive_fraction_max, bfr__, size__);
      bfr__ += IMC::deserialize(climb_fraction_max, bfr__, size__);
      bfr__ += IMC::deserialize(bank_max, bfr__, size__);
      bfr__ += IMC::deserialize(p_max, bfr__, size__);
      bfr__ += IMC::deserialize(pitch_min, bfr__, size__);
      bfr__ += IMC::deserialize(pitch_max, bfr__, size__);
      bfr__ += IMC::deserialize(q_max, bfr__, size__);
      bfr__ += IMC::deserialize(g_min, bfr__, size__);
      bfr__ += IMC::deserialize(g_max, bfr__, size__);
      bfr__ += IMC::deserialize(g_lat_max, bfr__, size__);
      bfr__ += IMC::deserialize(rpm_min, bfr__, size__);
      bfr__ += IMC::deserialize(rpm_max, bfr__, size__);
      bfr__ += IMC::deserialize(rpm_rate_max, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(long_accel, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(alt_max_msl, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dive_fraction_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(climb_fraction_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bank_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(p_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pitch_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pitch_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(q_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(g_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(g_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(g_lat_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rpm_rate_max, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleOperationalLimits::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleOperationalLimits";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 69;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "speed_min", speed_min, nindent__);
      IMC::toJSON(os__, "speed_max", speed_max, nindent__);
      IMC::toJSON(os__, "long_accel", long_accel, nindent__);
      IMC::toJSON(os__, "alt_max_msl", alt_max_msl, nindent__);
      IMC::toJSON(os__, "dive_fraction_max", dive_fraction_max, nindent__);
      IMC::toJSON(os__, "climb_fraction_max", climb_fraction_max, nindent__);
      IMC::toJSON(os__, "bank_max", bank_max, nindent__);
      IMC::toJSON(os__, "p_max", p_max, nindent__);
      IMC::toJSON(os__, "pitch_min", pitch_min, nindent__);
      IMC::toJSON(os__, "pitch_max", pitch_max, nindent__);
      IMC::toJSON(os__, "q_max", q_max, nindent__);
      IMC::toJSON(os__, "g_min", g_min, nindent__);
      IMC::toJSON(os__, "g_max", g_max, nindent__);
      IMC::toJSON(os__, "g_lat_max", g_lat_max, nindent__);
      IMC::toJSON(os__, "rpm_min", rpm_min, nindent__);
      IMC::toJSON(os__, "rpm_max", rpm_max, nindent__);
      IMC::toJSON(os__, "rpm_rate_max", rpm_rate_max, nindent__);
    }
  };
}

#endif
