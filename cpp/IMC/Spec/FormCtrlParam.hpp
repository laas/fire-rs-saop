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

#ifndef IMC_FORMCTRLPARAM_HPP_INCLUDED_
#define IMC_FORMCTRLPARAM_HPP_INCLUDED_

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
  //! Formation Control Parameters.
  class FormCtrlParam: public Message
  {
  public:
    //! Action.
    enum ActionEnum
    {
      //! Request.
      OP_REQ = 0,
      //! Set.
      OP_SET = 1,
      //! Report.
      OP_REP = 2
    };

    //! Action.
    uint8_t action;
    //! Longitudinal Gain.
    float longain;
    //! Lateral Gain.
    float latgain;
    //! Boundary Layer Thickness.
    uint32_t bondthick;
    //! Leader Gain.
    float leadgain;
    //! Deconfliction Gain.
    float deconflgain;

    static uint16_t
    getIdStatic(void)
    {
      return 820;
    }

    static FormCtrlParam*
    cast(Message* msg__)
    {
      return (FormCtrlParam*)msg__;
    }

    FormCtrlParam(void)
    {
      m_header.mgid = FormCtrlParam::getIdStatic();
      clear();
    }

    FormCtrlParam*
    clone(void) const
    {
      return new FormCtrlParam(*this);
    }

    void
    clear(void)
    {
      action = 0;
      longain = 0;
      latgain = 0;
      bondthick = 0;
      leadgain = 0;
      deconflgain = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormCtrlParam& other__ = static_cast<const FormCtrlParam&>(msg__);
      if (action != other__.action) return false;
      if (longain != other__.longain) return false;
      if (latgain != other__.latgain) return false;
      if (bondthick != other__.bondthick) return false;
      if (leadgain != other__.leadgain) return false;
      if (deconflgain != other__.deconflgain) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(action, ptr__);
      ptr__ += IMC::serialize(longain, ptr__);
      ptr__ += IMC::serialize(latgain, ptr__);
      ptr__ += IMC::serialize(bondthick, ptr__);
      ptr__ += IMC::serialize(leadgain, ptr__);
      ptr__ += IMC::serialize(deconflgain, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::deserialize(longain, bfr__, size__);
      bfr__ += IMC::deserialize(latgain, bfr__, size__);
      bfr__ += IMC::deserialize(bondthick, bfr__, size__);
      bfr__ += IMC::deserialize(leadgain, bfr__, size__);
      bfr__ += IMC::deserialize(deconflgain, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(action, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(longain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(latgain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(bondthick, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(leadgain, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(deconflgain, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormCtrlParam::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormCtrlParam";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 21;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "action", action, nindent__);
      IMC::toJSON(os__, "longain", longain, nindent__);
      IMC::toJSON(os__, "latgain", latgain, nindent__);
      IMC::toJSON(os__, "bondthick", bondthick, nindent__);
      IMC::toJSON(os__, "leadgain", leadgain, nindent__);
      IMC::toJSON(os__, "deconflgain", deconflgain, nindent__);
    }
  };
}

#endif
