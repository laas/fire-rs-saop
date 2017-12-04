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

#ifndef IMC_VEHICLESTATE_HPP_INCLUDED_
#define IMC_VEHICLESTATE_HPP_INCLUDED_

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
  //! Vehicle State.
  class VehicleState: public Message
  {
  public:
    //! Operation Mode.
    enum OperationModeEnum
    {
      //! Service.
      VS_SERVICE = 0,
      //! Calibration.
      VS_CALIBRATION = 1,
      //! Error.
      VS_ERROR = 2,
      //! Maneuver.
      VS_MANEUVER = 3,
      //! External Control.
      VS_EXTERNAL = 4,
      //! Boot.
      VS_BOOT = 5
    };

    //! Flags.
    enum FlagsBits
    {
      //! Maneuver Done.
      VFLG_MANEUVER_DONE = 0x01
    };

    //! Operation Mode.
    uint8_t op_mode;
    //! Errors -- Count.
    uint8_t error_count;
    //! Errors -- Entities.
    std::string error_ents;
    //! Maneuver -- Type.
    uint16_t maneuver_type;
    //! Maneuver -- Start Time.
    double maneuver_stime;
    //! Maneuver -- ETA.
    uint16_t maneuver_eta;
    //! Control Loops.
    uint32_t control_loops;
    //! Flags.
    uint8_t flags;
    //! Last Error -- Description.
    std::string last_error;
    //! Last Error -- Time.
    double last_error_time;

    static uint16_t
    getIdStatic(void)
    {
      return 500;
    }

    static VehicleState*
    cast(Message* msg__)
    {
      return (VehicleState*)msg__;
    }

    VehicleState(void)
    {
      m_header.mgid = VehicleState::getIdStatic();
      clear();
    }

    VehicleState*
    clone(void) const
    {
      return new VehicleState(*this);
    }

    void
    clear(void)
    {
      op_mode = 0;
      error_count = 0;
      error_ents.clear();
      maneuver_type = 0;
      maneuver_stime = 0;
      maneuver_eta = 0;
      control_loops = 0;
      flags = 0;
      last_error.clear();
      last_error_time = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleState& other__ = static_cast<const VehicleState&>(msg__);
      if (op_mode != other__.op_mode) return false;
      if (error_count != other__.error_count) return false;
      if (error_ents != other__.error_ents) return false;
      if (maneuver_type != other__.maneuver_type) return false;
      if (maneuver_stime != other__.maneuver_stime) return false;
      if (maneuver_eta != other__.maneuver_eta) return false;
      if (control_loops != other__.control_loops) return false;
      if (flags != other__.flags) return false;
      if (last_error != other__.last_error) return false;
      if (last_error_time != other__.last_error_time) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op_mode, ptr__);
      ptr__ += IMC::serialize(error_count, ptr__);
      ptr__ += IMC::serialize(error_ents, ptr__);
      ptr__ += IMC::serialize(maneuver_type, ptr__);
      ptr__ += IMC::serialize(maneuver_stime, ptr__);
      ptr__ += IMC::serialize(maneuver_eta, ptr__);
      ptr__ += IMC::serialize(control_loops, ptr__);
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += IMC::serialize(last_error, ptr__);
      ptr__ += IMC::serialize(last_error_time, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op_mode, bfr__, size__);
      bfr__ += IMC::deserialize(error_count, bfr__, size__);
      bfr__ += IMC::deserialize(error_ents, bfr__, size__);
      bfr__ += IMC::deserialize(maneuver_type, bfr__, size__);
      bfr__ += IMC::deserialize(maneuver_stime, bfr__, size__);
      bfr__ += IMC::deserialize(maneuver_eta, bfr__, size__);
      bfr__ += IMC::deserialize(control_loops, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::deserialize(last_error, bfr__, size__);
      bfr__ += IMC::deserialize(last_error_time, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op_mode, bfr__, size__);
      bfr__ += IMC::deserialize(error_count, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(error_ents, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(maneuver_type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(maneuver_stime, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(maneuver_eta, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(control_loops, bfr__, size__);
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(last_error, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(last_error_time, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 27;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(error_ents) + IMC::getSerializationSize(last_error);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op_mode", op_mode, nindent__);
      IMC::toJSON(os__, "error_count", error_count, nindent__);
      IMC::toJSON(os__, "error_ents", error_ents, nindent__);
      IMC::toJSON(os__, "maneuver_type", maneuver_type, nindent__);
      IMC::toJSON(os__, "maneuver_stime", maneuver_stime, nindent__);
      IMC::toJSON(os__, "maneuver_eta", maneuver_eta, nindent__);
      IMC::toJSON(os__, "control_loops", control_loops, nindent__);
      IMC::toJSON(os__, "flags", flags, nindent__);
      IMC::toJSON(os__, "last_error", last_error, nindent__);
      IMC::toJSON(os__, "last_error_time", last_error_time, nindent__);
    }
  };
}

#endif
