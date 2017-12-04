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

#ifndef IMC_TREXATTRIBUTE_HPP_INCLUDED_
#define IMC_TREXATTRIBUTE_HPP_INCLUDED_

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
  //! TREX Attribute.
  class TrexAttribute: public Message
  {
  public:
    //! Attribute type.
    enum AttributetypeEnum
    {
      //! Boolean Domain.
      TYPE_BOOL = 1,
      //! Integer Domain.
      TYPE_INT = 2,
      //! Float Domain.
      TYPE_FLOAT = 3,
      //! String Domain.
      TYPE_STRING = 4,
      //! Enumerated Domain.
      TYPE_ENUM = 5
    };

    //! Attribute Name.
    std::string name;
    //! Attribute type.
    uint8_t attr_type;
    //! Minimum.
    std::string min;
    //! Maximum.
    std::string max;

    static uint16_t
    getIdStatic(void)
    {
      return 656;
    }

    static TrexAttribute*
    cast(Message* msg__)
    {
      return (TrexAttribute*)msg__;
    }

    TrexAttribute(void)
    {
      m_header.mgid = TrexAttribute::getIdStatic();
      clear();
    }

    TrexAttribute*
    clone(void) const
    {
      return new TrexAttribute(*this);
    }

    void
    clear(void)
    {
      name.clear();
      attr_type = 0;
      min.clear();
      max.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::TrexAttribute& other__ = static_cast<const TrexAttribute&>(msg__);
      if (name != other__.name) return false;
      if (attr_type != other__.attr_type) return false;
      if (min != other__.min) return false;
      if (max != other__.max) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(name, ptr__);
      ptr__ += IMC::serialize(attr_type, ptr__);
      ptr__ += IMC::serialize(min, ptr__);
      ptr__ += IMC::serialize(max, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(attr_type, bfr__, size__);
      bfr__ += IMC::deserialize(min, bfr__, size__);
      bfr__ += IMC::deserialize(max, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(attr_type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(min, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(max, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return TrexAttribute::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "TrexAttribute";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name) + IMC::getSerializationSize(min) + IMC::getSerializationSize(max);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "name", name, nindent__);
      IMC::toJSON(os__, "attr_type", attr_type, nindent__);
      IMC::toJSON(os__, "min", min, nindent__);
      IMC::toJSON(os__, "max", max, nindent__);
    }
  };
}

#endif
