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

#ifndef IMC_DYNAMICSSIMPARAM_HPP_INCLUDED_
#define IMC_DYNAMICSSIMPARAM_HPP_INCLUDED_

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
  //! Dynamics Simulation Parameters.
  class DynamicsSimParam: public Message
  {
  public:
    //! Action on the Vehicle Simulation Parameters.
    enum ActionontheVehicleSimulationParametersEnum
    {
      //! Request.
      OP_REQUEST = 0,
      //! Set.
      OP_SET = 1,
      //! Report.
      OP_REPORT = 2
    };

    //! Action on the Vehicle Simulation Parameters.
    uint8_t op;
    //! TAS to Longitudinal Acceleration Gain.
    float tas2acc_pgain;
    //! Bank to Bank Rate Gain.
    float bank2p_pgain;

    static uint16_t
    getIdStatic(void)
    {
      return 53;
    }

    static DynamicsSimParam*
    cast(Message* msg__)
    {
      return (DynamicsSimParam*)msg__;
    }

    DynamicsSimParam(void)
    {
      m_header.mgid = DynamicsSimParam::getIdStatic();
      clear();
    }

    DynamicsSimParam*
    clone(void) const
    {
      return new DynamicsSimParam(*this);
    }

    void
    clear(void)
    {
      op = 0;
      tas2acc_pgain = 0;
      bank2p_pgain = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::DynamicsSimParam& other__ = static_cast<const DynamicsSimParam&>(msg__);
      if (op != other__.op) return false;
      if (tas2acc_pgain != other__.tas2acc_pgain) return false;
      if (bank2p_pgain != other__.bank2p_pgain) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(tas2acc_pgain, ptr__);
      ptr__ += IMC::serialize(bank2p_pgain, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(tas2acc_pgain, bfr__, size__);
      bfr__ += IMC::deserialize(bank2p_pgain, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(tas2acc_pgain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bank2p_pgain, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return DynamicsSimParam::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "DynamicsSimParam";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 9;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "tas2acc_pgain", tas2acc_pgain, nindent__);
      IMC::toJSON(os__, "bank2p_pgain", bank2p_pgain, nindent__);
    }
  };
}

#endif
