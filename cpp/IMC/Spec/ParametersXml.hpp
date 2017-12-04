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

#ifndef IMC_PARAMETERSXML_HPP_INCLUDED_
#define IMC_PARAMETERSXML_HPP_INCLUDED_

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
  //! Parameters XML.
  class ParametersXml: public Message
  {
  public:
    //! Locale.
    std::string locale;
    //! Configuration Data.
    std::vector<char> config;

    static uint16_t
    getIdStatic(void)
    {
      return 893;
    }

    static ParametersXml*
    cast(Message* msg__)
    {
      return (ParametersXml*)msg__;
    }

    ParametersXml(void)
    {
      m_header.mgid = ParametersXml::getIdStatic();
      clear();
    }

    ParametersXml*
    clone(void) const
    {
      return new ParametersXml(*this);
    }

    void
    clear(void)
    {
      locale.clear();
      config.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ParametersXml& other__ = static_cast<const ParametersXml&>(msg__);
      if (locale != other__.locale) return false;
      if (config != other__.config) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(locale, ptr__);
      ptr__ += IMC::serialize(config, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(locale, bfr__, size__);
      bfr__ += IMC::deserialize(config, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(locale, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(config, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ParametersXml::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ParametersXml";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(locale) + IMC::getSerializationSize(config);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "locale", locale, nindent__);
      IMC::toJSON(os__, "config", config, nindent__);
    }
  };
}

#endif
