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

#ifndef IMC_FORMATIONMONITOR_HPP_INCLUDED_
#define IMC_FORMATIONMONITOR_HPP_INCLUDED_

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
#include "../Spec/RelativeState.hpp"

namespace IMC
{
  //! Formation Monitoring Data.
  class FormationMonitor: public Message
  {
  public:
    //! Commanded X Acceleration (North).
    float ax_cmd;
    //! Commanded Y Acceleration (East).
    float ay_cmd;
    //! Commanded Z Acceleration (Down).
    float az_cmd;
    //! Desired X Acceleration (North).
    float ax_des;
    //! Desired Y Acceleration (East).
    float ay_des;
    //! Desired Z Acceleration (Down).
    float az_des;
    //! X Virtual Error (North).
    float virt_err_x;
    //! Y Virtual Error (East).
    float virt_err_y;
    //! Z Virtual Error (Down).
    float virt_err_z;
    //! X Sliding Surface Feedback (North).
    float surf_fdbk_x;
    //! Y Sliding Surface Feedback (East).
    float surf_fdbk_y;
    //! Z Sliding Surface Feedback (Down).
    float surf_fdbk_z;
    //! X Uncertainty Compensation (North).
    float surf_unkn_x;
    //! Y Uncertainty Compensation (East).
    float surf_unkn_y;
    //! Z Uncertainty Compensation (Down).
    float surf_unkn_z;
    //! X Convergence Deviation (North).
    float ss_x;
    //! Y Convergence Deviation (East).
    float ss_y;
    //! Z Convergence Deviation (Down).
    float ss_z;
    //! Relative State.
    MessageList<RelativeState> rel_state;

    static uint16_t
    getIdStatic(void)
    {
      return 481;
    }

    static FormationMonitor*
    cast(Message* msg__)
    {
      return (FormationMonitor*)msg__;
    }

    FormationMonitor(void)
    {
      m_header.mgid = FormationMonitor::getIdStatic();
      clear();
      rel_state.setParent(this);
    }

    FormationMonitor*
    clone(void) const
    {
      return new FormationMonitor(*this);
    }

    void
    clear(void)
    {
      ax_cmd = 0;
      ay_cmd = 0;
      az_cmd = 0;
      ax_des = 0;
      ay_des = 0;
      az_des = 0;
      virt_err_x = 0;
      virt_err_y = 0;
      virt_err_z = 0;
      surf_fdbk_x = 0;
      surf_fdbk_y = 0;
      surf_fdbk_z = 0;
      surf_unkn_x = 0;
      surf_unkn_y = 0;
      surf_unkn_z = 0;
      ss_x = 0;
      ss_y = 0;
      ss_z = 0;
      rel_state.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationMonitor& other__ = static_cast<const FormationMonitor&>(msg__);
      if (ax_cmd != other__.ax_cmd) return false;
      if (ay_cmd != other__.ay_cmd) return false;
      if (az_cmd != other__.az_cmd) return false;
      if (ax_des != other__.ax_des) return false;
      if (ay_des != other__.ay_des) return false;
      if (az_des != other__.az_des) return false;
      if (virt_err_x != other__.virt_err_x) return false;
      if (virt_err_y != other__.virt_err_y) return false;
      if (virt_err_z != other__.virt_err_z) return false;
      if (surf_fdbk_x != other__.surf_fdbk_x) return false;
      if (surf_fdbk_y != other__.surf_fdbk_y) return false;
      if (surf_fdbk_z != other__.surf_fdbk_z) return false;
      if (surf_unkn_x != other__.surf_unkn_x) return false;
      if (surf_unkn_y != other__.surf_unkn_y) return false;
      if (surf_unkn_z != other__.surf_unkn_z) return false;
      if (ss_x != other__.ss_x) return false;
      if (ss_y != other__.ss_y) return false;
      if (ss_z != other__.ss_z) return false;
      if (rel_state != other__.rel_state) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(ax_cmd, ptr__);
      ptr__ += IMC::serialize(ay_cmd, ptr__);
      ptr__ += IMC::serialize(az_cmd, ptr__);
      ptr__ += IMC::serialize(ax_des, ptr__);
      ptr__ += IMC::serialize(ay_des, ptr__);
      ptr__ += IMC::serialize(az_des, ptr__);
      ptr__ += IMC::serialize(virt_err_x, ptr__);
      ptr__ += IMC::serialize(virt_err_y, ptr__);
      ptr__ += IMC::serialize(virt_err_z, ptr__);
      ptr__ += IMC::serialize(surf_fdbk_x, ptr__);
      ptr__ += IMC::serialize(surf_fdbk_y, ptr__);
      ptr__ += IMC::serialize(surf_fdbk_z, ptr__);
      ptr__ += IMC::serialize(surf_unkn_x, ptr__);
      ptr__ += IMC::serialize(surf_unkn_y, ptr__);
      ptr__ += IMC::serialize(surf_unkn_z, ptr__);
      ptr__ += IMC::serialize(ss_x, ptr__);
      ptr__ += IMC::serialize(ss_y, ptr__);
      ptr__ += IMC::serialize(ss_z, ptr__);
      ptr__ += rel_state.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(ax_cmd, bfr__, size__);
      bfr__ += IMC::deserialize(ay_cmd, bfr__, size__);
      bfr__ += IMC::deserialize(az_cmd, bfr__, size__);
      bfr__ += IMC::deserialize(ax_des, bfr__, size__);
      bfr__ += IMC::deserialize(ay_des, bfr__, size__);
      bfr__ += IMC::deserialize(az_des, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_x, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_y, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_z, bfr__, size__);
      bfr__ += IMC::deserialize(surf_fdbk_x, bfr__, size__);
      bfr__ += IMC::deserialize(surf_fdbk_y, bfr__, size__);
      bfr__ += IMC::deserialize(surf_fdbk_z, bfr__, size__);
      bfr__ += IMC::deserialize(surf_unkn_x, bfr__, size__);
      bfr__ += IMC::deserialize(surf_unkn_y, bfr__, size__);
      bfr__ += IMC::deserialize(surf_unkn_z, bfr__, size__);
      bfr__ += IMC::deserialize(ss_x, bfr__, size__);
      bfr__ += IMC::deserialize(ss_y, bfr__, size__);
      bfr__ += IMC::deserialize(ss_z, bfr__, size__);
      bfr__ += rel_state.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(ax_cmd, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ay_cmd, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(az_cmd, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ax_des, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ay_des, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(az_des, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_fdbk_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_fdbk_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_fdbk_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_unkn_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_unkn_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(surf_unkn_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_z, bfr__, size__);
      bfr__ += rel_state.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationMonitor::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationMonitor";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 72;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return rel_state.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "ax_cmd", ax_cmd, nindent__);
      IMC::toJSON(os__, "ay_cmd", ay_cmd, nindent__);
      IMC::toJSON(os__, "az_cmd", az_cmd, nindent__);
      IMC::toJSON(os__, "ax_des", ax_des, nindent__);
      IMC::toJSON(os__, "ay_des", ay_des, nindent__);
      IMC::toJSON(os__, "az_des", az_des, nindent__);
      IMC::toJSON(os__, "virt_err_x", virt_err_x, nindent__);
      IMC::toJSON(os__, "virt_err_y", virt_err_y, nindent__);
      IMC::toJSON(os__, "virt_err_z", virt_err_z, nindent__);
      IMC::toJSON(os__, "surf_fdbk_x", surf_fdbk_x, nindent__);
      IMC::toJSON(os__, "surf_fdbk_y", surf_fdbk_y, nindent__);
      IMC::toJSON(os__, "surf_fdbk_z", surf_fdbk_z, nindent__);
      IMC::toJSON(os__, "surf_unkn_x", surf_unkn_x, nindent__);
      IMC::toJSON(os__, "surf_unkn_y", surf_unkn_y, nindent__);
      IMC::toJSON(os__, "surf_unkn_z", surf_unkn_z, nindent__);
      IMC::toJSON(os__, "ss_x", ss_x, nindent__);
      IMC::toJSON(os__, "ss_y", ss_y, nindent__);
      IMC::toJSON(os__, "ss_z", ss_z, nindent__);
      rel_state.toJSON(os__, "rel_state", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      rel_state.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      rel_state.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      rel_state.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      rel_state.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      rel_state.setDestinationEntity(value__);
    }
  };
}

#endif
