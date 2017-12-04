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

#ifndef IMC_FORMATIONEVALUATION_HPP_INCLUDED_
#define IMC_FORMATIONEVALUATION_HPP_INCLUDED_

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
#include "../Spec/FormationControlParams.hpp"

namespace IMC
{
  //! Formation Evaluation Data.
  class FormationEvaluation: public Message
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
      OP_STOP = 1,
      //! Ready.
      OP_READY = 2,
      //! Executing.
      OP_EXECUTING = 3,
      //! Failure.
      OP_FAILURE = 4
    };

    //! Type.
    uint8_t type;
    //! Operation.
    uint8_t op;
    //! Mean Position Error.
    float err_mean;
    //! Absolute Minimum Distance.
    float dist_min_abs;
    //! Mean Minimum Distance.
    float dist_min_mean;
    //! Mean Roll Rate.
    float roll_rate_mean;
    //! Evaluation Time.
    float time;
    //! Formation Control Parameters.
    InlineMessage<FormationControlParams> controlparams;

    static uint16_t
    getIdStatic(void)
    {
      return 823;
    }

    static FormationEvaluation*
    cast(Message* msg__)
    {
      return (FormationEvaluation*)msg__;
    }

    FormationEvaluation(void)
    {
      m_header.mgid = FormationEvaluation::getIdStatic();
      clear();
      controlparams.setParent(this);
    }

    FormationEvaluation*
    clone(void) const
    {
      return new FormationEvaluation(*this);
    }

    void
    clear(void)
    {
      type = 0;
      op = 0;
      err_mean = 0;
      dist_min_abs = 0;
      dist_min_mean = 0;
      roll_rate_mean = 0;
      time = 0;
      controlparams.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationEvaluation& other__ = static_cast<const FormationEvaluation&>(msg__);
      if (type != other__.type) return false;
      if (op != other__.op) return false;
      if (err_mean != other__.err_mean) return false;
      if (dist_min_abs != other__.dist_min_abs) return false;
      if (dist_min_mean != other__.dist_min_mean) return false;
      if (roll_rate_mean != other__.roll_rate_mean) return false;
      if (time != other__.time) return false;
      if (controlparams != other__.controlparams) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(type, ptr__);
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(err_mean, ptr__);
      ptr__ += IMC::serialize(dist_min_abs, ptr__);
      ptr__ += IMC::serialize(dist_min_mean, ptr__);
      ptr__ += IMC::serialize(roll_rate_mean, ptr__);
      ptr__ += IMC::serialize(time, ptr__);
      ptr__ += controlparams.serialize(ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(err_mean, bfr__, size__);
      bfr__ += IMC::deserialize(dist_min_abs, bfr__, size__);
      bfr__ += IMC::deserialize(dist_min_mean, bfr__, size__);
      bfr__ += IMC::deserialize(roll_rate_mean, bfr__, size__);
      bfr__ += IMC::deserialize(time, bfr__, size__);
      bfr__ += controlparams.deserialize(bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(type, bfr__, size__);
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(err_mean, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dist_min_abs, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(dist_min_mean, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(roll_rate_mean, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(time, bfr__, size__);
      bfr__ += controlparams.reverseDeserialize(bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationEvaluation::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationEvaluation";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 22;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return controlparams.getSerializationSize();
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "type", type, nindent__);
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "err_mean", err_mean, nindent__);
      IMC::toJSON(os__, "dist_min_abs", dist_min_abs, nindent__);
      IMC::toJSON(os__, "dist_min_mean", dist_min_mean, nindent__);
      IMC::toJSON(os__, "roll_rate_mean", roll_rate_mean, nindent__);
      IMC::toJSON(os__, "time", time, nindent__);
      controlparams.toJSON(os__, "controlparams", nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      if (!controlparams.isNull())
      {
        controlparams.get()->setTimeStamp(value__);
      }
    }

    void
    setSourceNested(uint16_t value__)
    {
      if (!controlparams.isNull())
      {
        controlparams.get()->setSource(value__);
      }
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      if (!controlparams.isNull())
      {
        controlparams.get()->setSourceEntity(value__);
      }
    }

    void
    setDestinationNested(uint16_t value__)
    {
      if (!controlparams.isNull())
      {
        controlparams.get()->setDestination(value__);
      }
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      if (!controlparams.isNull())
      {
        controlparams.get()->setDestinationEntity(value__);
      }
    }
  };
}

#endif
