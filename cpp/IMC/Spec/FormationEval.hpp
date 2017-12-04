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

#ifndef IMC_FORMATIONEVAL_HPP_INCLUDED_
#define IMC_FORMATIONEVAL_HPP_INCLUDED_

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
  //! Formation Evaluation Data.
  class FormationEval: public Message
  {
  public:
    //! Mean position error.
    float err_mean;
    //! Absolute minimum distance.
    float dist_min_abs;
    //! Mean minimum distance.
    float dist_min_mean;

    static uint16_t
    getIdStatic(void)
    {
      return 821;
    }

    static FormationEval*
    cast(Message* msg__)
    {
      return (FormationEval*)msg__;
    }

    FormationEval(void)
    {
      m_header.mgid = FormationEval::getIdStatic();
      clear();
    }

    FormationEval*
    clone(void) const
    {
      return new FormationEval(*this);
    }

    void
    clear(void)
    {
      err_mean = 0;
      dist_min_abs = 0;
      dist_min_mean = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationEval& other__ = static_cast<const FormationEval&>(msg__);
      if (err_mean != other__.err_mean) return false;
      if (dist_min_abs != other__.dist_min_abs) return false;
      if (dist_min_mean != other__.dist_min_mean) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(err_mean, ptr__);
      ptr__ += IMC::serialize(dist_min_abs, ptr__);
      ptr__ += IMC::serialize(dist_min_mean, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(err_mean, bfr__, size__);
      bfr__ += IMC::deserialize(dist_min_abs, bfr__, size__);
      bfr__ += IMC::deserialize(dist_min_mean, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(err_mean, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dist_min_abs, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dist_min_mean, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationEval::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationEval";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 12;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "err_mean", err_mean, nindent__);
      IMC::toJSON(os__, "dist_min_abs", dist_min_abs, nindent__);
      IMC::toJSON(os__, "dist_min_mean", dist_min_mean, nindent__);
    }
  };
}

#endif
