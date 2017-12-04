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

#ifndef IMC_FORMATIONPARAMETERS_HPP_INCLUDED_
#define IMC_FORMATIONPARAMETERS_HPP_INCLUDED_

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
#include "../Spec/VehicleFormationParticipant.hpp"

namespace IMC
{
  //! Formation Parameters.
  class FormationParameters: public Message
  {
  public:
    //! Formation Reference Frame.
    enum FormationReferenceFrameEnum
    {
      //! Earth Fixed.
      OP_EARTH_FIXED = 0,
      //! Path Fixed.
      OP_PATH_FIXED = 1,
      //! Path Curved.
      OP_PATH_CURVED = 2
    };

    //! Formation Name.
    std::string formation_name;
    //! Formation Reference Frame.
    uint8_t reference_frame;
    //! Formation Participants.
    MessageList<VehicleFormationParticipant> participants;
    //! Custom settings for formation.
    std::string custom;

    static uint16_t
    getIdStatic(void)
    {
      return 476;
    }

    static FormationParameters*
    cast(Message* msg__)
    {
      return (FormationParameters*)msg__;
    }

    FormationParameters(void)
    {
      m_header.mgid = FormationParameters::getIdStatic();
      clear();
      participants.setParent(this);
    }

    FormationParameters*
    clone(void) const
    {
      return new FormationParameters(*this);
    }

    void
    clear(void)
    {
      formation_name.clear();
      reference_frame = 0;
      participants.clear();
      custom.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::FormationParameters& other__ = static_cast<const FormationParameters&>(msg__);
      if (formation_name != other__.formation_name) return false;
      if (reference_frame != other__.reference_frame) return false;
      if (participants != other__.participants) return false;
      if (custom != other__.custom) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(formation_name, ptr__);
      ptr__ += IMC::serialize(reference_frame, ptr__);
      ptr__ += participants.serialize(ptr__);
      ptr__ += IMC::serialize(custom, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(formation_name, bfr__, size__);
      bfr__ += IMC::deserialize(reference_frame, bfr__, size__);
      bfr__ += participants.deserialize(bfr__, size__);
      bfr__ += IMC::deserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::reverseDeserialize(formation_name, bfr__, size__);
      bfr__ += IMC::deserialize(reference_frame, bfr__, size__);
      bfr__ += participants.reverseDeserialize(bfr__, size__);
      bfr__ += IMC::reverseDeserialize(custom, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return FormationParameters::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "FormationParameters";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 1;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(formation_name) + participants.getSerializationSize() + IMC::getSerializationSize(custom);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "formation_name", formation_name, nindent__);
      IMC::toJSON(os__, "reference_frame", reference_frame, nindent__);
      participants.toJSON(os__, "participants", nindent__);
      IMC::toJSON(os__, "custom", custom, nindent__);
    }

  protected:
    void
    setTimeStampNested(double value__)
    {
      participants.setTimeStamp(value__);
    }

    void
    setSourceNested(uint16_t value__)
    {
      participants.setSource(value__);
    }

    void
    setSourceEntityNested(uint8_t value__)
    {
      participants.setSourceEntity(value__);
    }

    void
    setDestinationNested(uint16_t value__)
    {
      participants.setDestination(value__);
    }

    void
    setDestinationEntityNested(uint8_t value__)
    {
      participants.setDestinationEntity(value__);
    }
  };
}

#endif
