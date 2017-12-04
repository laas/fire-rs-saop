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

#ifndef IMC_FORMATIONCONTROLPARAMS_HPP_INCLUDED_
#define IMC_FORMATIONCONTROLPARAMS_HPP_INCLUDED_

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
  //! Formation Control Parameters.
  class FormationControlParams: public Message
  {
  public:
    //! Action.
    enum ActionEnum
    {
      //! Request.
      OP_REQ = 0,
      //! Set.
      OP_SET = 1,
      //! Report.
      OP_REP = 2
    };

    //! Action.
    uint8_t action;
    //! Longitudinal Gain.
    float lon_gain;
    //! Lateral Gain.
    float lat_gain;
    //! Boundary Layer Thickness.
    float bond_thick;
    //! Leader Gain.
    float lead_gain;
    //! Deconfliction Gain.
    float deconfl_gain;
    //! Acceleration Switch Gain.
    float accel_switch_gain;
    //! Safety Distance.
    float safe_dist;
    //! Deconfliction Offset.
    float deconflict_offset;
    //! Acceleration Safety Margin.
    float accel_safe_margin;
    //! Maximum Longitudinal Acceleration.
    float accel_lim_x;

    static uint16_t
    getIdStatic(void)
    {
      return 822;
    }

    static FormationControlParams*
    cast(Message* msg__)
    {
      return (FormationControlParams*)msg__;
    }

    FormationControlParams(void)
    {
      m_header.mgid = FormationControlParams::getIdStatic();
      clear();
    }

    FormationControlParams*
    clone(void) const
    {
      return new FormationControlParams(*this);
    }

    void
    clear(void)
    {
      action = 0;
      lon_gain = 0;
      lat_gain = 0;
      bond_thick = 0;
      lead_gain = 0;
      deconfl_gain = 0;
      accel_switch_gain = 0;
      safe_dist = 0;
      deconflict_offset = 0;
      accel_safe_margin = 0;
      accel_lim_x = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationControlParams& other__ = static_cast<const FormationControlParams&>(msg__);
      if (action != other__.action) return false;
      if (lon_gain != other__.lon_gain) return false;
      if (lat_gain != other__.lat_gain) return false;
      if (bond_thick != other__.bond_thick) return false;
      if (lead_gain != other__.lead_gain) return false;
      if (deconfl_gain != other__.deconfl_gain) return false;
      if (accel_switch_gain != other__.accel_switch_gain) return false;
      if (safe_dist != other__.safe_dist) return false;
      if (deconflict_offset != other__.deconflict_offset) return false;
      if (accel_safe_margin != other__.accel_safe_margin) return false;
      if (accel_lim_x != other__.accel_lim_x) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(action, ptr__);
      ptr__ += IMC::serialize(lon_gain, ptr__);
      ptr__ += IMC::serialize(lat_gain, ptr__);
      ptr__ += IMC::serialize(bond_thick, ptr__);
      ptr__ += IMC::serialize(lead_gain, ptr__);
      ptr__ += IMC::serialize(deconfl_gain, ptr__);
      ptr__ += IMC::serialize(accel_switch_gain, ptr__);
      ptr__ += IMC::serialize(safe_dist, ptr__);
      ptr__ += IMC::serialize(deconflict_offset, ptr__);
      ptr__ += IMC::serialize(accel_safe_margin, ptr__);
      ptr__ += IMC::serialize(accel_lim_x, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::deserialize(lon_gain, bfr__, size__);
      bfr__ += IMC::deserialize(lat_gain, bfr__, size__);
      bfr__ += IMC::deserialize(bond_thick, bfr__, size__);
      bfr__ += IMC::deserialize(lead_gain, bfr__, size__);
      bfr__ += IMC::deserialize(deconfl_gain, bfr__, size__);
      bfr__ += IMC::deserialize(accel_switch_gain, bfr__, size__);
      bfr__ += IMC::deserialize(safe_dist, bfr__, size__);
      bfr__ += IMC::deserialize(deconflict_offset, bfr__, size__);
      bfr__ += IMC::deserialize(accel_safe_margin, bfr__, size__);
      bfr__ += IMC::deserialize(accel_lim_x, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon_gain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat_gain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bond_thick, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lead_gain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(deconfl_gain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(accel_switch_gain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(safe_dist, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(deconflict_offset, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(accel_safe_margin, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(accel_lim_x, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationControlParams::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationControlParams";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 41;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "action", action, nindent__);
      IMC::toJSON(os__, "lon_gain", lon_gain, nindent__);
      IMC::toJSON(os__, "lat_gain", lat_gain, nindent__);
      IMC::toJSON(os__, "bond_thick", bond_thick, nindent__);
      IMC::toJSON(os__, "lead_gain", lead_gain, nindent__);
      IMC::toJSON(os__, "deconfl_gain", deconfl_gain, nindent__);
      IMC::toJSON(os__, "accel_switch_gain", accel_switch_gain, nindent__);
      IMC::toJSON(os__, "safe_dist", safe_dist, nindent__);
      IMC::toJSON(os__, "deconflict_offset", deconflict_offset, nindent__);
      IMC::toJSON(os__, "accel_safe_margin", accel_safe_margin, nindent__);
      IMC::toJSON(os__, "accel_lim_x", accel_lim_x, nindent__);
    }
  };
}

#endif
