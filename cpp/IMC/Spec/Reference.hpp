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

#ifndef IMC_REFERENCE_HPP_INCLUDED_
#define IMC_REFERENCE_HPP_INCLUDED_

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
#include "../Spec/DesiredSpeed.hpp"
#include "../Spec/DesiredZ.hpp"

namespace IMC
{
  //! Reference To Follow.
  class Reference: public Message
  {
  public:
    //! Flags.
    enum FlagsBits
    {
      //! Use Location Reference.
      FLAG_LOCATION = 0x01,
      //! Use Speed Reference.
      FLAG_SPEED = 0x02,
      //! Use Z Reference.
      FLAG_Z = 0x04,
      //! Use Radius Reference.
      FLAG_RADIUS = 0x08,
      //! Use this Reference as Start Position for PathControler.
      FLAG_START_POINT = 0x10,
      //! Use Current Position as Start Position for PathControler.
      FLAG_DIRECT = 0x20,
      //! Flag Maneuver Completion.
      FLAG_MANDONE = 0x80
    };

    //! Flags.
    uint8_t flags;
    //! Speed Reference.
    InlineMessage<DesiredSpeed> speed;
    //! Z Reference.
    InlineMessage<DesiredZ> z;
    //! Latitude Reference.
    double lat;
    //! Longitude Reference.
    double lon;
    //! Radius.
    float radius;

    static uint16_t
    getIdStatic(void)
    {
      return 479;
    }

    static Reference*
    cast(Message* msg__)
    {
      return (Reference*)msg__;
    }

    Reference(void)
    {
      m_header.mgid = Reference::getIdStatic();
      clear();
      speed.setParent(this);
      z.setParent(this);
    }

    Reference*
    clone(void) const
    {
      return new Reference(*this);
    }

    void
    clear(void)
    {
      flags = 0;
      speed.clear();
      z.clear();
      lat = 0;
      lon = 0;
      radius = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Reference& other__ = static_cast<const Reference&>(msg__);
      if (flags != other__.flags) return false;
      if (speed != other__.speed) return false;
      if (z != other__.z) return false;
      if (lat != other__.lat) return false;
      if (lon != other__.lon) return false;
      if (radius != other__.radius) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(flags, ptr__);
      ptr__ += speed.serialize(ptr__);
      ptr__ += z.serialize(ptr__);
      ptr__ += IMC::serialize(lat, ptr__);
      ptr__ += IMC::serialize(lon, ptr__);
      ptr__ += IMC::serialize(radius, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += speed.deserialize(bfr__, size__);
      bfr__ += z.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(lat, bfr__, size__);
      bfr__ += IMC::deserialize(lon, bfr__, size__);
      bfr__ += IMC::deserialize(radius, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(flags, bfr__, size__);
      bfr__ += speed.reverseDeserialize(bfr__, size__);
      bfr__ += z.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lat, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(lon, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(radius, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Reference::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Reference";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 21;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return speed.getSerializationSize() + z.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "flags", flags, nindent__);
      speed.toJSON(os__, "speed", nindent__);
      z.toJSON(os__, "z", nindent__);
      IMC::toJSON(os__, "lat", lat, nindent__);
      IMC::toJSON(os__, "lon", lon, nindent__);
      IMC::toJSON(os__, "radius", radius, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!speed.isNull())
      {
        speed.get()->setTimeStamp(value__);
      }

      if (!z.isNull())
      {
        z.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!speed.isNull())
      {
        speed.get()->setSource(value__);
      }

      if (!z.isNull())
      {
        z.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!speed.isNull())
      {
        speed.get()->setSourceEntity(value__);
      }

      if (!z.isNull())
      {
        z.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!speed.isNull())
      {
        speed.get()->setDestination(value__);
      }

      if (!z.isNull())
      {
        z.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!speed.isNull())
      {
        speed.get()->setDestinationEntity(value__);
      }

      if (!z.isNull())
      {
        z.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
