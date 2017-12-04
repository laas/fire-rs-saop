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

#ifndef IMC_FORMATIONPLANEXECUTION_HPP_INCLUDED_
#define IMC_FORMATIONPLANEXECUTION_HPP_INCLUDED_

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
  //! Formation Plan Execution.
  class FormationPlanExecution: public Maneuver
  {
  public:
    //! Target Group Name.
    std::string group_name;
    //! Formation Name.
    std::string formation_name;
    //! Formation Plan ID.
    std::string plan_id;
    //! Plan Description.
    std::string description;
    //! Formation Leader Flight Airspeed.
    float leader_speed;
    //! Formation leader flight bank limit.
    float leader_bank_lim;
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
      return 477;
    }

    static FormationPlanExecution*
    cast(Message* msg__)
    {
      return (FormationPlanExecution*)msg__;
    }

    FormationPlanExecution(void)
    {
      m_header.mgid = FormationPlanExecution::getIdStatic();
      clear();
    }

    FormationPlanExecution*
    clone(void) const
    {
      return new FormationPlanExecution(*this);
    }

    void
    clear(void)
    {
      group_name.clear();
      formation_name.clear();
      plan_id.clear();
      description.clear();
      leader_speed = 0;
      leader_bank_lim = 0;
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
      const IMC::FormationPlanExecution& other__ = static_cast<const FormationPlanExecution&>(msg__);
      if (group_name != other__.group_name) return false;
      if (formation_name != other__.formation_name) return false;
      if (plan_id != other__.plan_id) return false;
      if (description != other__.description) return false;
      if (leader_speed != other__.leader_speed) return false;
      if (leader_bank_lim != other__.leader_bank_lim) return false;
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
      ptr__ += IMC::serialize(group_name, ptr__);
      ptr__ += IMC::serialize(formation_name, ptr__);
      ptr__ += IMC::serialize(plan_id, ptr__);
      ptr__ += IMC::serialize(description, ptr__);
      ptr__ += IMC::serialize(leader_speed, ptr__);
      ptr__ += IMC::serialize(leader_bank_lim, ptr__);
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
      bfr__ += IMC::deserialize(group_name, bfr__, size__);
      bfr__ += IMC::deserialize(formation_name, bfr__, size__);
      bfr__ += IMC::deserialize(plan_id, bfr__, size__);
      bfr__ += IMC::deserialize(description, bfr__, size__);
      bfr__ += IMC::deserialize(leader_speed, bfr__, size__);
      bfr__ += IMC::deserialize(leader_bank_lim, bfr__, size__);
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
      bfr__ += IMC::reverseDeserialize(group_name, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(formation_name, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(description, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_speed, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leader_bank_lim, bfr__, size__);
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
      return FormationPlanExecution::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationPlanExecution";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 30;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(group_name) + IMC::getSerializationSize(formation_name) + IMC::getSerializationSize(plan_id) + IMC::getSerializationSize(description) + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "group_name", group_name, nindent__);
      IMC::toJSON(os__, "formation_name", formation_name, nindent__);
      IMC::toJSON(os__, "plan_id", plan_id, nindent__);
      IMC::toJSON(os__, "description", description, nindent__);
      IMC::toJSON(os__, "leader_speed", leader_speed, nindent__);
      IMC::toJSON(os__, "leader_bank_lim", leader_bank_lim, nindent__);
      IMC::toJSON(os__, "pos_sim_err_lim", pos_sim_err_lim, nindent__);
      IMC::toJSON(os__, "pos_sim_err_wrn", pos_sim_err_wrn, nindent__);
      IMC::toJSON(os__, "pos_sim_err_timeout", pos_sim_err_timeout, nindent__);
      IMC::toJSON(os__, "converg_max", converg_max, nindent__);
      IMC::toJSON(os__, "converg_timeout", converg_timeout, nindent__);
      IMC::toJSON(os__, "comms_timeout", comms_timeout, nindent__);
      IMC::toJSON(os__, "turb_lim", turb_lim, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }
  };
}

#endif
