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

#ifndef IMC_TREXOBSERVATION_HPP_INCLUDED_
#define IMC_TREXOBSERVATION_HPP_INCLUDED_

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
  //! TREX Observation.
  class TrexObservation: public Message
  {
  public:
    //! Timeline.
    std::string timeline;
    //! Predicate.
    std::string predicate;
    //! Attributes.
    std::string attributes;

    static uint16_t
    getIdStatic(void)
    {
      return 651;
    }

    static TrexObservation*
    cast(Message* msg__)
    {
      return (TrexObservation*)msg__;
    }

    TrexObservation(void)
    {
      m_header.mgid = TrexObservation::getIdStatic();
      clear();
    }

    TrexObservation*
    clone(void) const
    {
      return new TrexObservation(*this);
    }

    void
    clear(void)
    {
      timeline.clear();
      predicate.clear();
      attributes.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TrexObservation& other__ = static_cast<const TrexObservation&>(msg__);
      if (timeline != other__.timeline) return false;
      if (predicate != other__.predicate) return false;
      if (attributes != other__.attributes) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(timeline, ptr__);
      ptr__ += IMC::serialize(predicate, ptr__);
      ptr__ += IMC::serialize(attributes, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(timeline, bfr__, size__);
      bfr__ += IMC::deserialize(predicate, bfr__, size__);
      bfr__ += IMC::deserialize(attributes, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(timeline, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(predicate, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(attributes, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexObservation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexObservation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(timeline) + IMC::getSerializationSize(predicate) + IMC::getSerializationSize(attributes);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "timeline", timeline, nindent__);
      IMC::toJSON(os__, "predicate", predicate, nindent__);
      IMC::toJSON(os__, "attributes", attributes, nindent__);
    }
  };
}

#endif
