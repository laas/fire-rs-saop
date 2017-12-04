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

#ifndef IMC_TREXPLAN_HPP_INCLUDED_
#define IMC_TREXPLAN_HPP_INCLUDED_

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
#include "../Spec/TrexToken.hpp"

namespace IMC
{
  //! TREX Plan.
  class TrexPlan: public Message
  {
  public:
    //! Reactor name.
    std::string reactor;
    //! Tokens.
    MessageList<TrexToken> tokens;

    static uint16_t
    getIdStatic(void)
    {
      return 658;
    }

    static TrexPlan*
    cast(Message* msg__)
    {
      return (TrexPlan*)msg__;
    }

    TrexPlan(void)
    {
      m_header.mgid = TrexPlan::getIdStatic();
      clear();
      tokens.setParent(this);
    }

    TrexPlan*
    clone(void) const
    {
      return new TrexPlan(*this);
    }

    void
    clear(void)
    {
      reactor.clear();
      tokens.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TrexPlan& other__ = static_cast<const TrexPlan&>(msg__);
      if (reactor != other__.reactor) return false;
      if (tokens != other__.tokens) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(reactor, ptr__);
      ptr__ += tokens.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(reactor, bfr__, size__);
      bfr__ += tokens.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(reactor, bfr__, size__);
      bfr__ += tokens.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexPlan::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexPlan";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(reactor) + tokens.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "reactor", reactor, nindent__);
      tokens.toJSON(os__, "tokens", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      tokens.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      tokens.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      tokens.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      tokens.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      tokens.setDestinationEntity(value__);
    }
  };
}

#endif
