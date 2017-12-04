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

#ifndef IMC_STATEREPORT_HPP_INCLUDED_
#define IMC_STATEREPORT_HPP_INCLUDED_

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
  //! State Report.
  class StateReport: public Message
  {
  public:
    //! Time Stamp.
    uint32_t stime;
    //! Latitude.
    float latitude;
    //! Longitude.
    float longitude;
    //! Altitude.
    uint16_t altitude;
    //! Depth.
    uint16_t depth;
    //! Heading.
    uint16_t heading;
    //! Speed.
    int16_t speed;
    //! Fuel.
    int8_t fuel;
    //! Execution State.
    int8_t exec_state;
    //! Plan Checksum.
    uint16_t plan_checksum;

    static uint16_t
    getIdStatic(void)
    {
      return 514;
    }

    static StateReport*
    cast(Message* msg__)
    {
      return (StateReport*)msg__;
    }

    StateReport(void)
    {
      m_header.mgid = StateReport::getIdStatic();
      clear();
    }

    StateReport*
    clone(void) const
    {
      return new StateReport(*this);
    }

    void
    clear(void)
    {
      stime = 0;
      latitude = 0;
      longitude = 0;
      altitude = 0;
      depth = 0;
      heading = 0;
      speed = 0;
      fuel = 0;
      exec_state = 0;
      plan_checksum = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::StateReport& other__ = static_cast<const StateReport&>(msg__);
      if (stime != other__.stime) return false;
      if (latitude != other__.latitude) return false;
      if (longitude != other__.longitude) return false;
      if (altitude != other__.altitude) return false;
      if (depth != other__.depth) return false;
      if (heading != other__.heading) return false;
      if (speed != other__.speed) return false;
      if (fuel != other__.fuel) return false;
      if (exec_state != other__.exec_state) return false;
      if (plan_checksum != other__.plan_checksum) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(stime, ptr__);
      ptr__ += IMC::serialize(latitude, ptr__);
      ptr__ += IMC::serialize(longitude, ptr__);
      ptr__ += IMC::serialize(altitude, ptr__);
      ptr__ += IMC::serialize(depth, ptr__);
      ptr__ += IMC::serialize(heading, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(fuel, ptr__);
      ptr__ += IMC::serialize(exec_state, ptr__);
      ptr__ += IMC::serialize(plan_checksum, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(stime, bfr__, size__);
      bfr__ += IMC::deserialize(latitude, bfr__, size__);
      bfr__ += IMC::deserialize(longitude, bfr__, size__);
      bfr__ += IMC::deserialize(altitude, bfr__, size__);
      bfr__ += IMC::deserialize(depth, bfr__, size__);
      bfr__ += IMC::deserialize(heading, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(fuel, bfr__, size__);
      bfr__ += IMC::deserialize(exec_state, bfr__, size__);
      bfr__ += IMC::deserialize(plan_checksum, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(stime, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(latitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(longitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(altitude, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(depth, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(heading, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(fuel, bfr__, size__);
      bfr__ += IMC::deserialize(exec_state, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(plan_checksum, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return StateReport::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "StateReport";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 24;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "stime", stime, nindent__);
      IMC::toJSON(os__, "latitude", latitude, nindent__);
      IMC::toJSON(os__, "longitude", longitude, nindent__);
      IMC::toJSON(os__, "altitude", altitude, nindent__);
      IMC::toJSON(os__, "depth", depth, nindent__);
      IMC::toJSON(os__, "heading", heading, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "fuel", fuel, nindent__);
      IMC::toJSON(os__, "exec_state", exec_state, nindent__);
      IMC::toJSON(os__, "plan_checksum", plan_checksum, nindent__);
    }
  };
}

#endif
