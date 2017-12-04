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

#ifndef IMC_TREXTOKEN_HPP_INCLUDED_
#define IMC_TREXTOKEN_HPP_INCLUDED_

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
#include "../Spec/TrexAttribute.hpp"

namespace IMC
{
  //! TREX Token.
  class TrexToken: public Message
  {
  public:
    //! Timeline.
    std::string timeline;
    //! Predicate.
    std::string predicate;
    //! Attributes.
    MessageList<TrexAttribute> attributes;

    static uint16_t
    getIdStatic(void)
    {
      return 657;
    }

    static TrexToken*
    cast(Message* msg__)
    {
      return (TrexToken*)msg__;
    }

    TrexToken(void)
    {
      m_header.mgid = TrexToken::getIdStatic();
      clear();
      attributes.setParent(this);
    }

    TrexToken*
    clone(void) const
    {
      return new TrexToken(*this);
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
      const IMC::TrexToken& other__ = static_cast<const TrexToken&>(msg__);
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
      ptr__ += attributes.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(timeline, bfr__, size__);
      bfr__ += IMC::deserialize(predicate, bfr__, size__);
      bfr__ += attributes.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(timeline, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(predicate, bfr__, size__);
      bfr__ += attributes.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexToken::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexToken";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(timeline) + IMC::getSerializationSize(predicate) + attributes.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "timeline", timeline, nindent__);
      IMC::toJSON(os__, "predicate", predicate, nindent__);
      attributes.toJSON(os__, "attributes", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      attributes.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      attributes.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      attributes.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      attributes.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      attributes.setDestinationEntity(value__);
    }
  };
}

#endif
