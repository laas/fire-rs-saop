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

#ifndef IMC_VEHICLEFORMATION_HPP_INCLUDED_
#define IMC_VEHICLEFORMATION_HPP_INCLUDED_

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
#include "../Spec/TrajectoryPoint.hpp"
#include "../Spec/VehicleFormationParticipant.hpp"
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Vehicle Formation.
  class VehicleFormation: public Maneuver
  {
  public:
    //! Latitude WGS-84.
    double lat;
    //! Longitude WGS-84.
    double lon;
    //! Z Reference.
    float z;
    //! Z Units.
    uint8_t z_units;
    //! Speed.
    float speed;
    //! Speed Units.
    uint8_t speed_units;
    //! Trajectory Points.
    MessageList<TrajectoryPoint> points;
    //! Formation Participants.
    MessageList<VehicleFormationParticipant> participants;
    //! Start Time.
    double start_time;
    //! Custom settings for maneuver.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 466;
    }

    static VehicleFormation*
    cast(Message* msg__)
    {
      return (VehicleFormation*)msg__;
    }

    VehicleFormation(void)
    {
      m_header.mgid = VehicleFormation::getIdStatic();
      clear();
      points.setParent(this);
      participants.setParent(this);
    }

    VehicleFormation*
    clone(void) const
    {
      return new VehicleFormation(*this);
    }

    void
    clear(void)
    {
      lat = 0;
      lon = 0;
      z = 0;
      z_units = 0;
      speed = 0;
      speed_units = 0;
      points.clear();
      participants.clear();
      start_time = 0;
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleFormation& other__ = static_cast<const VehicleFormation&>(msg__);
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (z != other__.z) return false;
      if (z_units != other__.z_units) return false;
      if (speed != other__.speed) return false;
      if (speed_units != other__.speed_units) return false;
      if (points != other__.points) return false;
      if (participants != other__.participants) return false;
      if (start_time != other__.start_time) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(z, ptr__);
      ptr__ += IMC::serialize(z_units, ptr__);
      ptr__ += IMC::serialize(speed, ptr__);
      ptr__ += IMC::serialize(speed_units, ptr__);
      ptr__ += points.serialize(ptr__);
      ptr__ += participants.serialize(ptr__);
      ptr__ += IMC::serialize(start_time, ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::deserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += points.deserialize(bfr__, size__);
      bfr__ += participants.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(start_time, bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(z, bfr__, size__);
      bfr__ += IMC::deserialize(z_units, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(speed, bfr__, size__);
      bfr__ += IMC::deserialize(speed_units, bfr__, size__);
      bfr__ += points.reverseDeserialize(bfr__, size__);
      bfr__ += participants.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(start_time, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleFormation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleFormation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 34;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return points.getSerializationSize() + participants.getSerializationSize() + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "z", z, nindent__);
      IMC::toJSON(os__, "z_units", z_units, nindent__);
      IMC::toJSON(os__, "speed", speed, nindent__);
      IMC::toJSON(os__, "speed_units", speed_units, nindent__);
      points.toJSON(os__, "points", nindent__);
      participants.toJSON(os__, "participants", nindent__);
      IMC::toJSON(os__, "start_time", start_time, nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      points.setTimeStamp(value__);

      participants.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      points.setSource(value__);

      participants.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      points.setSourceEntity(value__);

      participants.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      points.setDestination(value__);

      participants.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      points.setDestinationEntity(value__);

      participants.setDestinationEntity(value__);
    }
  };
}

#endif
