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

#ifndef IMC_FORMATIONSTATE_HPP_INCLUDED_
#define IMC_FORMATIONSTATE_HPP_INCLUDED_

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
  //! Formation Tracking State.
  class FormationState: public Message
  {
  public:
    //! Type.
    enum TypeEnum
    {
      //! Request.
      FC_REQUEST = 0,
      //! Report.
      FC_REPORT = 1
    };

    //! Operation.
    enum OperationEnum
    {
      //! Start.
      OP_START = 0,
      //! Stop.
      OP_STOP = 1
    };

    //! Position Mismatch Monitor.
    enum PositionMismatchMonitorEnum
    {
      //! Ok.
      POS_OK = 0,
      //! Warning threshold.
      POS_WRN = 1,
      //! Limit threshold.
      POS_LIM = 2
    };

    //! Communications Monitor.
    enum CommunicationsMonitorEnum
    {
      //! Ok.
      COMMS_OK = 0,
      //! Timeout.
      COMMS_TIMEOUT = 1
    };

    //! Convergence.
    enum ConvergenceEnum
    {
      //! Ok.
      CONV_OK = 0,
      //! Timeout.
      CONV_TIMEOUT = 1
    };

    //! Type.
    uint8_t type;
    //! Operation.
    uint8_t op;
    //! Position Mismatch.
    float possimerr;
    //! Convergence.
    float converg;
    //! Stream Turbulence.
    float turbulence;
    //! Position Mismatch Monitor.
    uint8_t possimmon;
    //! Communications Monitor.
    uint8_t commmon;
    //! Convergence.
    uint8_t convergmon;

    static uint16_t
    getIdStatic(void)
    {
      return 512;
    }

    static FormationState*
    cast(Message* msg__)
    {
      return (FormationState*)msg__;
    }

    FormationState(void)
    {
      m_header.mgid = FormationState::getIdStatic();
      clear();
    }

    FormationState*
    clone(void) const
    {
      return new FormationState(*this);
    }

    void
    clear(void)
    {
      type = 0;
      op = 0;
      possimerr = 0;
      converg = 0;
      turbulence = 0;
      possimmon = 0;
      commmon = 0;
      convergmon = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationState& other__ = static_cast<const FormationState&>(msg__);
      if (type != other__.type) return false;
      if (op != other__.op) return false;
      if (possimerr != other__.possimerr) return false;
      if (converg != other__.converg) return false;
      if (turbulence != other__.turbulence) return false;
      if (possimmon != other__.possimmon) return false;
      if (commmon != other__.commmon) return false;
      if (convergmon != other__.convergmon) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(possimerr, ptr__);
      ptr__ += IMC::serialize(converg, ptr__);
      ptr__ += IMC::serialize(turbulence, ptr__);
      ptr__ += IMC::serialize(possimmon, ptr__);
      ptr__ += IMC::serialize(commmon, ptr__);
      ptr__ += IMC::serialize(convergmon, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(possimerr, bfr__, size__);
      bfr__ += IMC::deserialize(converg, bfr__, size__);
      bfr__ += IMC::deserialize(turbulence, bfr__, size__);
      bfr__ += IMC::deserialize(possimmon, bfr__, size__);
      bfr__ += IMC::deserialize(commmon, bfr__, size__);
      bfr__ += IMC::deserialize(convergmon, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(possimerr, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(converg, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(turbulence, bfr__, size__);
      bfr__ += IMC::deserialize(possimmon, bfr__, size__);
      bfr__ += IMC::deserialize(commmon, bfr__, size__);
      bfr__ += IMC::deserialize(convergmon, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationState::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationState";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 17;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "possimerr", possimerr, nindent__);
      IMC::toJSON(os__, "converg", converg, nindent__);
      IMC::toJSON(os__, "turbulence", turbulence, nindent__);
      IMC::toJSON(os__, "possimmon", possimmon, nindent__);
      IMC::toJSON(os__, "commmon", commmon, nindent__);
      IMC::toJSON(os__, "convergmon", convergmon, nindent__);
    }
  };
}

#endif
