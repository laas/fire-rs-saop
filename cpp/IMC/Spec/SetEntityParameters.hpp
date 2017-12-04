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

#ifndef IMC_SETENTITYPARAMETERS_HPP_INCLUDED_
#define IMC_SETENTITYPARAMETERS_HPP_INCLUDED_

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
#include "../Spec/EntityParameter.hpp"

namespace IMC
{
  //! SetEntityParameters.
  class SetEntityParameters: public Message
  {
  public:
    //! Entity Name.
    std::string name;
    //! Parameters.
    MessageList<EntityParameter> params;

    static uint16_t
    getIdStatic(void)
    {
      return 804;
    }

    static SetEntityParameters*
    cast(Message* msg__)
    {
      return (SetEntityParameters*)msg__;
    }

    SetEntityParameters(void)
    {
      m_header.mgid = SetEntityParameters::getIdStatic();
      clear();
      params.setParent(this);
    }

    SetEntityParameters*
    clone(void) const
    {
      return new SetEntityParameters(*this);
    }

    void
    clear(void)
    {
      name.clear();
      params.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::SetEntityParameters& other__ = static_cast<const SetEntityParameters&>(msg__);
      if (name != other__.name) return false;
      if (params != other__.params) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(name, ptr__);
      ptr__ += params.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(name, bfr__, size__);
      bfr__ += params.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(name, bfr__, size__);
      bfr__ += params.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return SetEntityParameters::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "SetEntityParameters";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(name) + params.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "name", name, nindent__);
      params.toJSON(os__, "params", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      params.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      params.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      params.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      params.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      params.setDestinationEntity(value__);
    }
  };
}

#endif
