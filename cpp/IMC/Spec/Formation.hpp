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

#ifndef IMC_FORMATION_HPP_INCLUDED_
#define IMC_FORMATION_HPP_INCLUDED_

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
#include "../Spec/VehicleFormationParticipant.hpp"

namespace IMC
{
  //! Formation.
  class Formation: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Request.
      FC_REQUEST = 0,
      //! Report.
      FC_REPORT = 1
    };

    //! Operation.
    enum OperationEnum
    {
      //! Start.
      OP_START = 0,
      //! Stop.
      OP_STOP = 1,
      //! Ready.
      OP_READY = 2,
      //! Executing.
      OP_EXECUTING = 3,
      //! Failure.
      OP_FAILURE = 4
    };

    //! Formation Reference Frame.
    enum FormationReferenceFrameEnum
    {
      //! Earth Fixed.
      OP_EARTH_FIXED = 0,
      //! Path Fixed.
      OP_PATH_FIXED = 1,
      //! Path Curved.
      OP_PATH_CURVED = 2
    };

    //! Formation Name.
    std::string formation_name;
    //! Type.
    uint8_t type;
    //! Operation.
    uint8_t op;
    //! Target Group Name.
    std::string group_name;
    //! Formation Plan ID.
    std::string plan_id;
    //! Plan Description.
    std::string description;
    //! Formation Reference Frame.
    uint8_t reference_frame;
    //! Formation Participants.
    MessageList<VehicleFormationParticipant> participants;
    //! Formation Leader Bank Limit.
    float leader_bank_lim;
    //! Formation Leader Minimum Speed.
    float leader_speed_min;
    //! Formation Leader Maximum Speed.
    float leader_speed_max;
    //! Formation Leader Minimum Altitude.
    float leader_alt_min;
    //! Formation Leader Maximum Altitude.
    float leader_alt_max;
    //! Position mismatch limit.
    float pos_sim_err_lim;
    //! Position mismatch threshold.
    float pos_sim_err_wrn;
    //! Position mismatch time-out.
    uint16_t pos_sim_err_timeout;
    //! Convergence threshold.
    float converg_max;
    //! Convergence time-out.
    uint16_t converg_timeout;
    //! Communications time-out.
    uint16_t comms_timeout;
    //! Turbulence limit.
    float turb_lim;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 484;
    }

    static Formation*
    cast(Message* msg__)
    {
      return (Formation*)msg__;
    }

    Formation(void)
    {
      m_header.mgid = Formation::getIdStatic();
      clear();
      participants.setParent(this);
    }

    Formation*
    clone(void) const
    {
      return new Formation(*this);
    }

    void
    clear(void)
    {
      formation_name.clear();
      type = 0;
      op = 0;
      group_name.clear();
      plan_id.clear();
      description.clear();
      reference_frame = 0;
      participants.clear();
      leader_bank_lim = 0;
      leader_speed_min = 0;
      leader_speed_max = 0;
      leader_alt_min = 0;
      leader_alt_max = 0;
      pos_sim_err_lim = 0;
      pos_sim_err_wrn = 0;
      pos_sim_err_timeout = 0;
      converg_max = 0;
      converg_timeout = 0;
      comms_timeout = 0;
      turb_lim = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Formation& other__ = static_cast<const Formation&>(msg__);
      if (formation_name != other__.formation_name) return false;
      if (type != other__.type) return false;
      if (op != other__.op) return false;
      if (group_name != other__.group_name) return false;
      if (plan_id != other__.plan_id) return false;
      if (description != other__.description) return false;
      if (reference_frame != other__.reference_frame) return false;
      if (participants != other__.participants) return false;
      if (leader_bank_lim != other__.leader_bank_lim) return false;
      if (leader_speed_min != other__.leader_speed_min) return false;
      if (leader_speed_max != other__.leader_speed_max) return false;
      if (leader_alt_min != other__.leader_alt_min) return false;
      if (leader_alt_max != other__.leader_alt_max) return false;
      if (pos_sim_err_lim != other__.pos_sim_err_lim) return false;
      if (pos_sim_err_wrn != other__.pos_sim_err_wrn) return false;
      if (pos_sim_err_timeout != other__.pos_sim_err_timeout) return false;
      if (converg_max != other__.converg_max) return false;
      if (converg_timeout != other__.converg_timeout) return false;
      if (comms_timeout != other__.comms_timeout) return false;
      if (turb_lim != other__.turb_lim) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(formation_name, ptr__);
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(group_name, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(description, ptr__);
      ptr__ += IMC::serialize(reference_frame, ptr__);
      ptr__ += participants.serialize(ptr__);
      ptr__ += IMC::serialize(leader_bank_lim, ptr__);
      ptr__ += IMC::serialize(leader_speed_min, ptr__);
      ptr__ += IMC::serialize(leader_speed_max, ptr__);
      ptr__ += IMC::serialize(leader_alt_min, ptr__);
      ptr__ += IMC::serialize(leader_alt_max, ptr__);
      ptr__ += IMC::serialize(pos_sim_err_lim, ptr__);
      ptr__ += IMC::serialize(pos_sim_err_wrn, ptr__);
      ptr__ += IMC::serialize(pos_sim_err_timeout, ptr__);
      ptr__ += IMC::serialize(converg_max, ptr__);
      ptr__ += IMC::serialize(converg_timeout, ptr__);
      ptr__ += IMC::serialize(comms_timeout, ptr__);
      ptr__ += IMC::serialize(turb_lim, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(formation_name, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(group_name, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(description, bfr__, size__);
      bfr__ += IMC::deserialize(reference_frame, bfr__, size__);
      bfr__ += participants.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(leader_bank_lim, bfr__, size__);
      bfr__ += IMC::deserialize(leader_speed_min, bfr__, size__);
      bfr__ += IMC::deserialize(leader_speed_max, bfr__, size__);
      bfr__ += IMC::deserialize(leader_alt_min, bfr__, size__);
      bfr__ += IMC::deserialize(leader_alt_max, bfr__, size__);
      bfr__ += IMC::deserialize(pos_sim_err_lim, bfr__, size__);
      bfr__ += IMC::deserialize(pos_sim_err_wrn, bfr__, size__);
      bfr__ += IMC::deserialize(pos_sim_err_timeout, bfr__, size__);
      bfr__ += IMC::deserialize(converg_max, bfr__, size__);
      bfr__ += IMC::deserialize(converg_timeout, bfr__, size__);
      bfr__ += IMC::deserialize(comms_timeout, bfr__, size__);
      bfr__ += IMC::deserialize(turb_lim, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(formation_name, bfr__, size__);
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(group_name, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(description, bfr__, size__);
      bfr__ += IMC::deserialize(reference_frame, bfr__, size__);
      bfr__ += participants.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_bank_lim, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_speed_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_speed_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_alt_min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_alt_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pos_sim_err_lim, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pos_sim_err_wrn, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(pos_sim_err_timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(converg_max, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(converg_timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(comms_timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(turb_lim, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Formation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Formation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 45;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(formation_name) + IMC::getSerializationSize(group_name) + IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(description) + participants.getSerializationSize() + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "formation_name", formation_name, nindent__);
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "group_name", group_name, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "description", description, nindent__);
      IMC::toJSON(os__, "reference_frame", reference_frame, nindent__);
      participants.toJSON(os__, "participants", nindent__);
      IMC::toJSON(os__, "leader_bank_lim", leader_bank_lim, nindent__);
      IMC::toJSON(os__, "leader_speed_min", leader_speed_min, nindent__);
      IMC::toJSON(os__, "leader_speed_max", leader_speed_max, nindent__);
      IMC::toJSON(os__, "leader_alt_min", leader_alt_min, nindent__);
      IMC::toJSON(os__, "leader_alt_max", leader_alt_max, nindent__);
      IMC::toJSON(os__, "pos_sim_err_lim", pos_sim_err_lim, nindent__);
      IMC::toJSON(os__, "pos_sim_err_wrn", pos_sim_err_wrn, nindent__);
      IMC::toJSON(os__, "pos_sim_err_timeout", pos_sim_err_timeout, nindent__);
      IMC::toJSON(os__, "converg_max", converg_max, nindent__);
      IMC::toJSON(os__, "converg_timeout", converg_timeout, nindent__);
      IMC::toJSON(os__, "comms_timeout", comms_timeout, nindent__);
      IMC::toJSON(os__, "turb_lim", turb_lim, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      participants.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      participants.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      participants.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      participants.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      participants.setDestinationEntity(value__);
    }
  };
}

#endif
