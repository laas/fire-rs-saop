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

#ifndef IMC_ACOUSTICLINK_HPP_INCLUDED_
#define IMC_ACOUSTICLINK_HPP_INCLUDED_

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
  //! Acoustic Link Quality.
  class AcousticLink: public Message
  {
  public:
    //! Peer Name.
    std::string peer;
    //! Received Signal Strength Indicator.
    float rssi;
    //! Signal Integrity Level.
    uint16_t integrity;

    static uint16_t
    getIdStatic(void)
    {
      return 214;
    }

    static AcousticLink*
    cast(Message* msg__)
    {
      return (AcousticLink*)msg__;
    }

    AcousticLink(void)
    {
      m_header.mgid = AcousticLink::getIdStatic();
      clear();
    }

    AcousticLink*
    clone(void) const
    {
      return new AcousticLink(*this);
    }

    void
    clear(void)
    {
      peer.clear();
      rssi = 0;
      integrity = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AcousticLink& other__ = static_cast<const AcousticLink&>(msg__);
      if (peer != other__.peer) return false;
      if (rssi != other__.rssi) return false;
      if (integrity != other__.integrity) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(peer, ptr__);
      ptr__ += IMC::serialize(rssi, ptr__);
      ptr__ += IMC::serialize(integrity, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(peer, bfr__, size__);
      bfr__ += IMC::deserialize(rssi, bfr__, size__);
      bfr__ += IMC::deserialize(integrity, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(peer, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(rssi, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(integrity, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AcousticLink::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AcousticLink";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 6;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(peer);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "peer", peer, nindent__);
      IMC::toJSON(os__, "rssi", rssi, nindent__);
      IMC::toJSON(os__, "integrity", integrity, nindent__);
    }
  };
}

#endif
