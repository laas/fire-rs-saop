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

#ifndef IMC_IOEVENT_HPP_INCLUDED_
#define IMC_IOEVENT_HPP_INCLUDED_

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
  //! I/O Event.
  class IoEvent: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Input Available.
      IOV_TYPE_INPUT = 1,
      //! Input Error.
      IOV_TYPE_INPUT_ERROR = 2
    };

    //! Type.
    uint8_t type;
    //! Error Message.
    std::string error;

    static uint16_t
    getIdStatic(void)
    {
      return 813;
    }

    static IoEvent*
    cast(Message* msg__)
    {
      return (IoEvent*)msg__;
    }

    IoEvent(void)
    {
      m_header.mgid = IoEvent::getIdStatic();
      clear();
    }

    IoEvent*
    clone(void) const
    {
      return new IoEvent(*this);
    }

    void
    clear(void)
    {
      type = 0;
      error.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::IoEvent& other__ = static_cast<const IoEvent&>(msg__);
      if (type != other__.type) return false;
      if (error != other__.error) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(error, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(error, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return IoEvent::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "IoEvent";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(error);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "error", error, nindent__);
    }
  };
}

#endif
