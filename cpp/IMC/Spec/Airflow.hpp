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

#ifndef IMC_AIRFLOW_HPP_INCLUDED_
#define IMC_AIRFLOW_HPP_INCLUDED_

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
  //! Airflow.
  class Airflow: public Message
  {
  public:
    //! Airspeed.
    float va;
    //! Angle of attack.
    float aoa;
    //! Sideslip angle.
    float ssa;

    static uint16_t
    getIdStatic(void)
    {
      return 363;
    }

    static Airflow*
    cast(Message* msg__)
    {
      return (Airflow*)msg__;
    }

    Airflow(void)
    {
      m_header.mgid = Airflow::getIdStatic();
      clear();
    }

    Airflow*
    clone(void) const
    {
      return new Airflow(*this);
    }

    void
    clear(void)
    {
      va = 0;
      aoa = 0;
      ssa = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::Airflow& other__ = static_cast<const Airflow&>(msg__);
      if (va != other__.va) return false;
      if (aoa != other__.aoa) return false;
      if (ssa != other__.ssa) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(va, ptr__);
      ptr__ += IMC::serialize(aoa, ptr__);
      ptr__ += IMC::serialize(ssa, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(va, bfr__, size__);
      bfr__ += IMC::deserialize(aoa, bfr__, size__);
      bfr__ += IMC::deserialize(ssa, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(va, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(aoa, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(ssa, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return Airflow::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "Airflow";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "va", va, nindent__);
      IMC::toJSON(os__, "aoa", aoa, nindent__);
      IMC::toJSON(os__, "ssa", ssa, nindent__);
    }
  };
}

#endif
