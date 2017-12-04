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

#ifndef IMC_FOLLOWREFERENCE_HPP_INCLUDED_
#define IMC_FOLLOWREFERENCE_HPP_INCLUDED_

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
  //! Follow Reference Maneuver.
  class FollowReference: public Maneuver
  {
  public:
    //! Controlling Source.
    uint16_t control_src;
    //! Controlling Entity.
    uint8_t control_ent;
    //! Reference Update Timeout.
    float timeout;
    //! Loiter Radius.
    float loiter_radius;
    //! Altitude Interval.
    float altitude_interval;

    static uint16_t
    getIdStatic(void)
    {
      return 478;
    }

    static FollowReference*
    cast(Message* msg__)
    {
      return (FollowReference*)msg__;
    }

    FollowReference(void)
    {
      m_header.mgid = FollowReference::getIdStatic();
      clear();
    }

    FollowReference*
    clone(void) const
    {
      return new FollowReference(*this);
    }

    void
    clear(void)
    {
      control_src = 0;
      control_ent = 0;
      timeout = 0;
      loiter_radius = 0;
      altitude_interval = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FollowReference& other__ = static_cast<const FollowReference&>(msg__);
      if (control_src != other__.control_src) return false;
      if (control_ent != other__.control_ent) return false;
      if (timeout != other__.timeout) return false;
      if (loiter_radius != other__.loiter_radius) return false;
      if (altitude_interval != other__.altitude_interval) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(control_src, ptr__);
      ptr__ += IMC::serialize(control_ent, ptr__);
      ptr__ += IMC::serialize(timeout, ptr__);
      ptr__ += IMC::serialize(loiter_radius, ptr__);
      ptr__ += IMC::serialize(altitude_interval, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += IMC::deserialize(timeout, bfr__, size__);
      bfr__ += IMC::deserialize(loiter_radius, bfr__, size__);
      bfr__ += IMC::deserialize(altitude_interval, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(control_src, bfr__, size__);
      bfr__ += IMC::deserialize(control_ent, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(timeout, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(loiter_radius, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(altitude_interval, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FollowReference::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FollowReference";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 15;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "control_src", control_src, nindent__);
      IMC::toJSON(os__, "control_ent", control_ent, nindent__);
      IMC::toJSON(os__, "timeout", timeout, nindent__);
      IMC::toJSON(os__, "loiter_radius", loiter_radius, nindent__);
      IMC::toJSON(os__, "altitude_interval", altitude_interval, nindent__);
    }
  };
}

#endif
