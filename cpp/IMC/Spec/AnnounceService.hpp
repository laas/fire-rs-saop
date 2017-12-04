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

#ifndef IMC_ANNOUNCESERVICE_HPP_INCLUDED_
#define IMC_ANNOUNCESERVICE_HPP_INCLUDED_

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
  //! Announce Service.
  class AnnounceService: public Message
  {
  public:
    //! ServiceType.
    enum ServiceTypeBits
    {
      //! External.
      SRV_TYPE_EXTERNAL = 0x01,
      //! Local.
      SRV_TYPE_LOCAL = 0x02
    };

    //! Service.
    std::string service;
    //! ServiceType.
    uint8_t service_type;

    static uint16_t
    getIdStatic(void)
    {
      return 152;
    }

    static AnnounceService*
    cast(Message* msg__)
    {
      return (AnnounceService*)msg__;
    }

    AnnounceService(void)
    {
      m_header.mgid = AnnounceService::getIdStatic();
      clear();
    }

    AnnounceService*
    clone(void) const
    {
      return new AnnounceService(*this);
    }

    void
    clear(void)
    {
      service.clear();
      service_type = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::AnnounceService& other__ = static_cast<const AnnounceService&>(msg__);
      if (service != other__.service) return false;
      if (service_type != other__.service_type) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(service, ptr__);
      ptr__ += IMC::serialize(service_type, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(service, bfr__, size__);
      bfr__ += IMC::deserialize(service_type, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(service, bfr__, size__);
      bfr__ += IMC::deserialize(service_type, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return AnnounceService::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "AnnounceService";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(service);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "service", service, nindent__);
      IMC::toJSON(os__, "service_type", service_type, nindent__);
    }
  };
}

#endif
