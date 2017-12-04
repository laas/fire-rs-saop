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

#ifndef IMC_USBLCONFIG_HPP_INCLUDED_
#define IMC_USBLCONFIG_HPP_INCLUDED_

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
#include "../Spec/UsblModem.hpp"

namespace IMC
{
  //! USBL Configuration.
  class UsblConfig: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Set LBL Configuration.
      OP_SET_CFG = 0,
      //! Retrieve LBL Configuration.
      OP_GET_CFG = 1,
      //! Reply to a GET command.
      OP_CUR_CFG = 2
    };

    //! Operation.
    uint8_t op;
    //! Modems.
    MessageList<UsblModem> modems;

    static uint16_t
    getIdStatic(void)
    {
      return 902;
    }

    static UsblConfig*
    cast(Message* msg__)
    {
      return (UsblConfig*)msg__;
    }

    UsblConfig(void)
    {
      m_header.mgid = UsblConfig::getIdStatic();
      clear();
      modems.setParent(this);
    }

    UsblConfig*
    clone(void) const
    {
      return new UsblConfig(*this);
    }

    void
    clear(void)
    {
      op = 0;
      modems.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::UsblConfig& other__ = static_cast<const UsblConfig&>(msg__);
      if (op != other__.op) return false;
      if (modems != other__.modems) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += modems.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += modems.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += modems.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return UsblConfig::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "UsblConfig";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return modems.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      modems.toJSON(os__, "modems", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      modems.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      modems.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      modems.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      modems.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      modems.setDestinationEntity(value__);
    }
  };
}

#endif
