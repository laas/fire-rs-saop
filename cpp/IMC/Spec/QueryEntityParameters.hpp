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

#ifndef IMC_QUERYENTITYPARAMETERS_HPP_INCLUDED_
#define IMC_QUERYENTITYPARAMETERS_HPP_INCLUDED_

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
  //! QueryEntityParameters.
  class QueryEntityParameters: public Message
  {
  public:
    //! Entity Name.
    std::string name;
    //! Visibility.
    std::string visibility;
    //! Scope.
    std::string scope;

    static uint16_t
    getIdStatic(void)
    {
      return 803;
    }

    static QueryEntityParameters*
    cast(Message* msg__)
    {
      return (QueryEntityParameters*)msg__;
    }

    QueryEntityParameters(void)
    {
      m_header.mgid = QueryEntityParameters::getIdStatic();
      clear();
    }

    QueryEntityParameters*
    clone(void) const
    {
      return new QueryEntityParameters(*this);
    }

    void
    clear(void)
    {
      name.clear();
      visibility.clear();
      scope.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::QueryEntityParameters& other__ = static_cast<const QueryEntityParameters&>(msg__);
      if (name != other__.name) return false;
      if (visibility != other__.visibility) return false;
      if (scope != other__.scope) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(name, ptr__);
      ptr__ += IMC::serialize(visibility, ptr__);
      ptr__ += IMC::serialize(scope, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(name, bfr__, size__);
      bfr__ += IMC::deserialize(visibility, bfr__, size__);
      bfr__ += IMC::deserialize(scope, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(visibility, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(scope, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return QueryEntityParameters::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "QueryEntityParameters";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name) + IMC::getSerializationSize(visibility) + IMC::getSerializationSize(scope);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "name", name, nindent__);
      IMC::toJSON(os__, "visibility", visibility, nindent__);
      IMC::toJSON(os__, "scope", scope, nindent__);
    }
  };
}

#endif
