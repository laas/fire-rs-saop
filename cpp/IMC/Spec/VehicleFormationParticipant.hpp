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

#ifndef IMC_VEHICLEFORMATIONPARTICIPANT_HPP_INCLUDED_
#define IMC_VEHICLEFORMATIONPARTICIPANT_HPP_INCLUDED_

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
  //! Vehicle Formation Participant.
  class VehicleFormationParticipant: public Message
  {
  public:
    //! ID (IMC address).
    uint16_t vid;
    //! Formation offset -- Along-track.
    float off_x;
    //! Formation offset -- Cross-track.
    float off_y;
    //! Formation offset -- Depth/Altitude.
    float off_z;

    static uint16_t
    getIdStatic(void)
    {
      return 467;
    }

    static VehicleFormationParticipant*
    cast(Message* msg__)
    {
      return (VehicleFormationParticipant*)msg__;
    }

    VehicleFormationParticipant(void)
    {
      m_header.mgid = VehicleFormationParticipant::getIdStatic();
      clear();
    }

    VehicleFormationParticipant*
    clone(void) const
    {
      return new VehicleFormationParticipant(*this);
    }

    void
    clear(void)
    {
      vid = 0;
      off_x = 0;
      off_y = 0;
      off_z = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleFormationParticipant& other__ = static_cast<const VehicleFormationParticipant&>(msg__);
      if (vid != other__.vid) return false;
      if (off_x != other__.off_x) return false;
      if (off_y != other__.off_y) return false;
      if (off_z != other__.off_z) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(vid, ptr__);
      ptr__ += IMC::serialize(off_x, ptr__);
      ptr__ += IMC::serialize(off_y, ptr__);
      ptr__ += IMC::serialize(off_z, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(vid, bfr__, size__);
      bfr__ += IMC::deserialize(off_x, bfr__, size__);
      bfr__ += IMC::deserialize(off_y, bfr__, size__);
      bfr__ += IMC::deserialize(off_z, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(vid, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(off_x, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(off_y, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(off_z, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleFormationParticipant::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleFormationParticipant";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 14;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "vid", vid, nindent__);
      IMC::toJSON(os__, "off_x", off_x, nindent__);
      IMC::toJSON(os__, "off_y", off_y, nindent__);
      IMC::toJSON(os__, "off_z", off_z, nindent__);
    }
  };
}

#endif
