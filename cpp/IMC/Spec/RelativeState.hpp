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

#ifndef IMC_RELATIVESTATE_HPP_INCLUDED_
#define IMC_RELATIVESTATE_HPP_INCLUDED_

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
  //! Relative State.
  class RelativeState: public Message
  {
  public:
    //! System Identifier.
    std::string s_id;
    //! Distance.
    float dist;
    //! Position Error.
    float err;
    //! Control Importance.
    float ctrl_imp;
    //! Relative Direction X (North).
    float rel_dir_x;
    //! Relative Direction Y (East).
    float rel_dir_y;
    //! Relative Direction Z (Down).
    float rel_dir_z;
    //! X Position Error (North).
    float err_x;
    //! Y Position Error (East).
    float err_y;
    //! Z Position Error (Down).
    float err_z;
    //! X Position Error In Relative Frame (North).
    float rf_err_x;
    //! Y Position Error In Relative Frame (East).
    float rf_err_y;
    //! Z Position Error In Relative Frame (Down).
    float rf_err_z;
    //! X Velocity Error In Relative Frame (North).
    float rf_err_vx;
    //! Y Velocity Error In Relative Frame (East).
    float rf_err_vy;
    //! Z Velocity Error In Relative Frame (Down).
    float rf_err_vz;
    //! X Convergence Deviation (North).
    float ss_x;
    //! Y Convergence Deviation (East).
    float ss_y;
    //! Z Convergence Deviation (Down).
    float ss_z;
    //! X Virtual Error (North).
    float virt_err_x;
    //! Y Virtual Error (East).
    float virt_err_y;
    //! Z Virtual Error (Down).
    float virt_err_z;

    static uint16_t
    getIdStatic(void)
    {
      return 482;
    }

    static RelativeState*
    cast(Message* msg__)
    {
      return (RelativeState*)msg__;
    }

    RelativeState(void)
    {
      m_header.mgid = RelativeState::getIdStatic();
      clear();
    }

    RelativeState*
    clone(void) const
    {
      return new RelativeState(*this);
    }

    void
    clear(void)
    {
      s_id.clear();
      dist = 0;
      err = 0;
      ctrl_imp = 0;
      rel_dir_x = 0;
      rel_dir_y = 0;
      rel_dir_z = 0;
      err_x = 0;
      err_y = 0;
      err_z = 0;
      rf_err_x = 0;
      rf_err_y = 0;
      rf_err_z = 0;
      rf_err_vx = 0;
      rf_err_vy = 0;
      rf_err_vz = 0;
      ss_x = 0;
      ss_y = 0;
      ss_z = 0;
      virt_err_x = 0;
      virt_err_y = 0;
      virt_err_z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::RelativeState& other__ = static_cast<const RelativeState&>(msg__);
      if (s_id != other__.s_id) return false;
      if (dist != other__.dist) return false;
      if (err != other__.err) return false;
      if (ctrl_imp != other__.ctrl_imp) return false;
      if (rel_dir_x != other__.rel_dir_x) return false;
      if (rel_dir_y != other__.rel_dir_y) return false;
      if (rel_dir_z != other__.rel_dir_z) return false;
      if (err_x != other__.err_x) return false;
      if (err_y != other__.err_y) return false;
      if (err_z != other__.err_z) return false;
      if (rf_err_x != other__.rf_err_x) return false;
      if (rf_err_y != other__.rf_err_y) return false;
      if (rf_err_z != other__.rf_err_z) return false;
      if (rf_err_vx != other__.rf_err_vx) return false;
      if (rf_err_vy != other__.rf_err_vy) return false;
      if (rf_err_vz != other__.rf_err_vz) return false;
      if (ss_x != other__.ss_x) return false;
      if (ss_y != other__.ss_y) return false;
      if (ss_z != other__.ss_z) return false;
      if (virt_err_x != other__.virt_err_x) return false;
      if (virt_err_y != other__.virt_err_y) return false;
      if (virt_err_z != other__.virt_err_z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(s_id, ptr__);
      ptr__ += IMC::serialize(dist, ptr__);
      ptr__ += IMC::serialize(err, ptr__);
      ptr__ += IMC::serialize(ctrl_imp, ptr__);
      ptr__ += IMC::serialize(rel_dir_x, ptr__);
      ptr__ += IMC::serialize(rel_dir_y, ptr__);
      ptr__ += IMC::serialize(rel_dir_z, ptr__);
      ptr__ += IMC::serialize(err_x, ptr__);
      ptr__ += IMC::serialize(err_y, ptr__);
      ptr__ += IMC::serialize(err_z, ptr__);
      ptr__ += IMC::serialize(rf_err_x, ptr__);
      ptr__ += IMC::serialize(rf_err_y, ptr__);
      ptr__ += IMC::serialize(rf_err_z, ptr__);
      ptr__ += IMC::serialize(rf_err_vx, ptr__);
      ptr__ += IMC::serialize(rf_err_vy, ptr__);
      ptr__ += IMC::serialize(rf_err_vz, ptr__);
      ptr__ += IMC::serialize(ss_x, ptr__);
      ptr__ += IMC::serialize(ss_y, ptr__);
      ptr__ += IMC::serialize(ss_z, ptr__);
      ptr__ += IMC::serialize(virt_err_x, ptr__);
      ptr__ += IMC::serialize(virt_err_y, ptr__);
      ptr__ += IMC::serialize(virt_err_z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(s_id, bfr__, size__);
      bfr__ += IMC::deserialize(dist, bfr__, size__);
      bfr__ += IMC::deserialize(err, bfr__, size__);
      bfr__ += IMC::deserialize(ctrl_imp, bfr__, size__);
      bfr__ += IMC::deserialize(rel_dir_x, bfr__, size__);
      bfr__ += IMC::deserialize(rel_dir_y, bfr__, size__);
      bfr__ += IMC::deserialize(rel_dir_z, bfr__, size__);
      bfr__ += IMC::deserialize(err_x, bfr__, size__);
      bfr__ += IMC::deserialize(err_y, bfr__, size__);
      bfr__ += IMC::deserialize(err_z, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_x, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_y, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_z, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_vx, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_vy, bfr__, size__);
      bfr__ += IMC::deserialize(rf_err_vz, bfr__, size__);
      bfr__ += IMC::deserialize(ss_x, bfr__, size__);
      bfr__ += IMC::deserialize(ss_y, bfr__, size__);
      bfr__ += IMC::deserialize(ss_z, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_x, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_y, bfr__, size__);
      bfr__ += IMC::deserialize(virt_err_z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(s_id, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dist, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(err, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ctrl_imp, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rel_dir_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rel_dir_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rel_dir_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(err_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(err_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(err_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_vx, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_vy, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rf_err_vz, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ss_z, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(virt_err_z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return RelativeState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "RelativeState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 84;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(s_id);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "s_id", s_id, nindent__);
      IMC::toJSON(os__, "dist", dist, nindent__);
      IMC::toJSON(os__, "err", err, nindent__);
      IMC::toJSON(os__, "ctrl_imp", ctrl_imp, nindent__);
      IMC::toJSON(os__, "rel_dir_x", rel_dir_x, nindent__);
      IMC::toJSON(os__, "rel_dir_y", rel_dir_y, nindent__);
      IMC::toJSON(os__, "rel_dir_z", rel_dir_z, nindent__);
      IMC::toJSON(os__, "err_x", err_x, nindent__);
      IMC::toJSON(os__, "err_y", err_y, nindent__);
      IMC::toJSON(os__, "err_z", err_z, nindent__);
      IMC::toJSON(os__, "rf_err_x", rf_err_x, nindent__);
      IMC::toJSON(os__, "rf_err_y", rf_err_y, nindent__);
      IMC::toJSON(os__, "rf_err_z", rf_err_z, nindent__);
      IMC::toJSON(os__, "rf_err_vx", rf_err_vx, nindent__);
      IMC::toJSON(os__, "rf_err_vy", rf_err_vy, nindent__);
      IMC::toJSON(os__, "rf_err_vz", rf_err_vz, nindent__);
      IMC::toJSON(os__, "ss_x", ss_x, nindent__);
      IMC::toJSON(os__, "ss_y", ss_y, nindent__);
      IMC::toJSON(os__, "ss_z", ss_z, nindent__);
      IMC::toJSON(os__, "virt_err_x", virt_err_x, nindent__);
      IMC::toJSON(os__, "virt_err_y", virt_err_y, nindent__);
      IMC::toJSON(os__, "virt_err_z", virt_err_z, nindent__);
    }
  };
}

#endif
