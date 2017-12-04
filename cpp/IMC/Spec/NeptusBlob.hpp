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

#ifndef IMC_NEPTUSBLOB_HPP_INCLUDED_
#define IMC_NEPTUSBLOB_HPP_INCLUDED_

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
  //! Neptus Blob.
  class NeptusBlob: public Message
  {
  public:
    //! ContentType.
    std::string content_type;
    //! Content.
    std::vector<char> content;

    static uint16_t
    getIdStatic(void)
    {
      return 888;
    }

    static NeptusBlob*
    cast(Message* msg__)
    {
      return (NeptusBlob*)msg__;
    }

    NeptusBlob(void)
    {
      m_header.mgid = NeptusBlob::getIdStatic();
      clear();
    }

    NeptusBlob*
    clone(void) const
    {
      return new NeptusBlob(*this);
    }

    void
    clear(void)
    {
      content_type.clear();
      content.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::NeptusBlob& other__ = static_cast<const NeptusBlob&>(msg__);
      if (content_type != other__.content_type) return false;
      if (content != other__.content) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(content_type, ptr__);
      ptr__ += IMC::serialize(content, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(content_type, bfr__, size__);
      bfr__ += IMC::deserialize(content, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(content_type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(content, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return NeptusBlob::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "NeptusBlob";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(content_type) + IMC::getSerializationSize(content);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "content_type", content_type, nindent__);
      IMC::toJSON(os__, "content", content, nindent__);
    }
  };
}

#endif
