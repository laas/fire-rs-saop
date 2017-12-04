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

#ifndef IMC_NAVIGATIONDATA_HPP_INCLUDED_
#define IMC_NAVIGATIONDATA_HPP_INCLUDED_

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
  //! Navigation Data.
  class NavigationData: public Message
  {
  public:
    //! Yaw Bias.
    float bias_psi;
    //! Gyro. Yaw Rate Bias.
    float bias_r;
    //! Course Over Ground.
    float cog;
    //! Continuous Yaw.
    float cyaw;
    //! GPS Rejection Filter Level.
    float lbl_rej_level;
    //! LBL Rejection Filter Level.
    float gps_rej_level;
    //! Variance - Custom Variable X.
    float custom_x;
    //! Variance - Custom Variable Y.
    float custom_y;
    //! Variance - Custom Variable Z.
    float custom_z;

    static uint16_t
    getIdStatic(void)
    {
      return 355;
    }

    static NavigationData*
    cast(Message* msg__)
    {
      return (NavigationData*)msg__;
    }

    NavigationData(void)
    {
      m_header.mgid = NavigationData::getIdStatic();
      clear();
    }

    NavigationData*
    clone(void) const
    {
      return new NavigationData(*this);
    }

    void
    clear(void)
    {
      bias_psi = 0;
      bias_r = 0;
      cog = 0;
      cyaw = 0;
      lbl_rej_level = 0;
      gps_rej_level = 0;
      custom_x = 0;
      custom_y = 0;
      custom_z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::NavigationData& other__ = static_cast<const NavigationData&>(msg__);
      if (bias_psi != other__.bias_psi) return false;
      if (bias_r != other__.bias_r) return false;
      if (cog != other__.cog) return false;
      if (cyaw != other__.cyaw) return false;
      if (lbl_rej_level != other__.lbl_rej_level) return false;
      if (gps_rej_level != other__.gps_rej_level) return false;
      if (custom_x != other__.custom_x) return false;
      if (custom_y != other__.custom_y) return false;
      if (custom_z != other__.custom_z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(bias_psi, ptr__);
      ptr__ += IMC::serialize(bias_r, ptr__);
      ptr__ += IMC::serialize(cog, ptr__);
      ptr__ += IMC::serialize(cyaw, ptr__);
      ptr__ += IMC::serialize(lbl_rej_level, ptr__);
      ptr__ += IMC::serialize(gps_rej_level, ptr__);
      ptr__ += IMC::serialize(custom_x, ptr__);
      ptr__ += IMC::serialize(custom_y, ptr__);
      ptr__ += IMC::serialize(custom_z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(bias_psi, bfr__, size__);
      bfr__ += IMC::deserialize(bias_r, bfr__, size__);
      bfr__ += IMC::deserialize(cog, bfr__, size__);
      bfr__ += IMC::deserialize(cyaw, bfr__, size__);
      bfr__ += IMC::deserialize(lbl_rej_level, bfr__, size__);
      bfr__ += IMC::deserialize(gps_rej_level, bfr__, size__);
      bfr__ += IMC::deserialize(custom_x, bfr__, size__);
      bfr__ += IMC::deserialize(custom_y, bfr__, size__);
      bfr__ += IMC::deserialize(custom_z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(bias_psi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bias_r, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cog, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(cyaw, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lbl_rej_level, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(gps_rej_level, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom_z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return NavigationData::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "NavigationData";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 36;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "bias_psi", bias_psi, nindent__);
      IMC::toJSON(os__, "bias_r", bias_r, nindent__);
      IMC::toJSON(os__, "cog", cog, nindent__);
      IMC::toJSON(os__, "cyaw", cyaw, nindent__);
      IMC::toJSON(os__, "lbl_rej_level", lbl_rej_level, nindent__);
      IMC::toJSON(os__, "gps_rej_level", gps_rej_level, nindent__);
      IMC::toJSON(os__, "custom_x", custom_x, nindent__);
      IMC::toJSON(os__, "custom_y", custom_y, nindent__);
      IMC::toJSON(os__, "custom_z", custom_z, nindent__);
    }
  };
}

#endif
