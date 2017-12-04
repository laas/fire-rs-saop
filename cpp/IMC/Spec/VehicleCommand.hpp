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

#ifndef IMC_VEHICLECOMMAND_HPP_INCLUDED_
#define IMC_VEHICLECOMMAND_HPP_INCLUDED_

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
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Vehicle Command.
  class VehicleCommand: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Request.
      VC_REQUEST = 0,
      //! Reply -- Success.
      VC_SUCCESS = 1,
      //! Reply -- In Progress.
      VC_IN_PROGRESS = 2,
      //! Reply -- Failure.
      VC_FAILURE = 3
    };

    //! Command.
    enum CommandEnum
    {
      //! Execute Maneuver.
      VC_EXEC_MANEUVER = 0,
      //! Stop Maneuver.
      VC_STOP_MANEUVER = 1,
      //! Start Calibration.
      VC_START_CALIBRATION = 2,
      //! Stop Calibration.
      VC_STOP_CALIBRATION = 3
    };

    //! Type.
    uint8_t type;
    //! Request ID.
    uint16_t request_id;
    //! Command.
    uint8_t command;
    //! Maneuver.
    InlineMessage<Maneuver> maneuver;
    //! Calibration Time.
    uint16_t calib_time;
    //! Info.
    std::string info;

    static uint16_t
    getIdStatic(void)
    {
      return 501;
    }

    static VehicleCommand*
    cast(Message* msg__)
    {
      return (VehicleCommand*)msg__;
    }

    VehicleCommand(void)
    {
      m_header.mgid = VehicleCommand::getIdStatic();
      clear();
      maneuver.setParent(this);
    }

    VehicleCommand*
    clone(void) const
    {
      return new VehicleCommand(*this);
    }

    void
    clear(void)
    {
      type = 0;
      request_id = 0;
      command = 0;
      maneuver.clear();
      calib_time = 0;
      info.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleCommand& other__ = static_cast<const VehicleCommand&>(msg__);
      if (type != other__.type) return false;
      if (request_id != other__.request_id) return false;
      if (command != other__.command) return false;
      if (maneuver != other__.maneuver) return false;
      if (calib_time != other__.calib_time) return false;
      if (info != other__.info) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(request_id, ptr__);
      ptr__ += IMC::serialize(command, ptr__);
      ptr__ += maneuver.serialize(ptr__);
      ptr__ += IMC::serialize(calib_time, ptr__);
      ptr__ += IMC::serialize(info, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(request_id, bfr__, size__);
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += maneuver.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(calib_time, bfr__, size__);
      bfr__ += IMC::deserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(request_id, bfr__, size__);
      bfr__ += IMC::deserialize(command, bfr__, size__);
      bfr__ += maneuver.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(calib_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(info, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleCommand::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleCommand";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return maneuver.getSerializationSize() + IMC::getSerializationSize(info);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "request_id", request_id, nindent__);
      IMC::toJSON(os__, "command", command, nindent__);
      maneuver.toJSON(os__, "maneuver", nindent__);
      IMC::toJSON(os__, "calib_time", calib_time, nindent__);
      IMC::toJSON(os__, "info", info, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!maneuver.isNull())
      {
        maneuver.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!maneuver.isNull())
      {
        maneuver.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!maneuver.isNull())
      {
        maneuver.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!maneuver.isNull())
      {
        maneuver.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!maneuver.isNull())
      {
        maneuver.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
