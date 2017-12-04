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

#ifndef IMC_VEHICLELINKS_HPP_INCLUDED_
#define IMC_VEHICLELINKS_HPP_INCLUDED_

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
#include "../Spec/Announce.hpp"

namespace IMC
{
  //! Vehicle Links.
  class VehicleLinks: public Message
  {
  public:
    //! Local Name.
    std::string localname;
    //! Active Links.
    MessageList<Announce> links;

    static uint16_t
    getIdStatic(void)
    {
      return 650;
    }

    static VehicleLinks*
    cast(Message* msg__)
    {
      return (VehicleLinks*)msg__;
    }

    VehicleLinks(void)
    {
      m_header.mgid = VehicleLinks::getIdStatic();
      clear();
      links.setParent(this);
    }

    VehicleLinks*
    clone(void) const
    {
      return new VehicleLinks(*this);
    }

    void
    clear(void)
    {
      localname.clear();
      links.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::VehicleLinks& other__ = static_cast<const VehicleLinks&>(msg__);
      if (localname != other__.localname) return false;
      if (links != other__.links) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(localname, ptr__);
      ptr__ += links.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(localname, bfr__, size__);
      bfr__ += links.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(localname, bfr__, size__);
      bfr__ += links.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return VehicleLinks::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "VehicleLinks";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(localname) + links.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "localname", localname, nindent__);
      links.toJSON(os__, "links", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      links.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      links.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      links.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      links.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      links.setDestinationEntity(value__);
    }
  };
}

#endif
