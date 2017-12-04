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

#ifndef IMC_QUERYPOWERCHANNELSTATE_HPP_INCLUDED_
#define IMC_QUERYPOWERCHANNELSTATE_HPP_INCLUDED_

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
  //! Query Power Channel State.
  class QueryPowerChannelState: public Message
  {
  public:

    static uint16_t
    getIdStatic(void)
    {
      return 310;
    }

    static QueryPowerChannelState*
    cast(Message* msg__)
    {
      return (QueryPowerChannelState*)msg__;
    }

    QueryPowerChannelState(void)
    {
      m_header.mgid = QueryPowerChannelState::getIdStatic();
      clear();
    }

    QueryPowerChannelState*
    clone(void) const
    {
      return new QueryPowerChannelState(*this);
    }

    void
    clear(void)
    {
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      return bfr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      (void)bfr__;
      (void)size__;
      return 0;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      (void)bfr__;
      (void)size__;
      return 0;
    }

    uint16_t
    getId(void) const
    {
      return QueryPowerChannelState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "QueryPowerChannelState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }
  };
}

#endif
