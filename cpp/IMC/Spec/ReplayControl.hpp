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

#ifndef IMC_REPLAYCONTROL_HPP_INCLUDED_
#define IMC_REPLAYCONTROL_HPP_INCLUDED_

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
  //! Replay Control.
  class ReplayControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Start.
      ROP_START = 0,
      //! Stop.
      ROP_STOP = 1,
      //! Pause.
      ROP_PAUSE = 2,
      //! Resume.
      ROP_RESUME = 3
    };

    //! Operation.
    uint8_t op;
    //! File To Replay.
    std::string file;

    static uint16_t
    getIdStatic(void)
    {
      return 105;
    }

    static ReplayControl*
    cast(Message* msg__)
    {
      return (ReplayControl*)msg__;
    }

    ReplayControl(void)
    {
      m_header.mgid = ReplayControl::getIdStatic();
      clear();
    }

    ReplayControl*
    clone(void) const
    {
      return new ReplayControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      file.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ReplayControl& other__ = static_cast<const ReplayControl&>(msg__);
      if (op != other__.op) return false;
      if (file != other__.file) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(file, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(file, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(file, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ReplayControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ReplayControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(file);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "file", file, nindent__);
    }
  };
}

#endif
