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

#ifndef IMC_IMAGETXSETTINGS_HPP_INCLUDED_
#define IMC_IMAGETXSETTINGS_HPP_INCLUDED_

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
  //! Image Transmission Settings.
  class ImageTxSettings: public Message
  {
  public:
    //! Frames Per Second.
    uint8_t fps;
    //! Quality.
    uint8_t quality;
    //! Repetitions.
    uint8_t reps;
    //! Target Size.
    uint8_t tsize;

    static uint16_t
    getIdStatic(void)
    {
      return 703;
    }

    static ImageTxSettings*
    cast(Message* msg__)
    {
      return (ImageTxSettings*)msg__;
    }

    ImageTxSettings(void)
    {
      m_header.mgid = ImageTxSettings::getIdStatic();
      clear();
    }

    ImageTxSettings*
    clone(void) const
    {
      return new ImageTxSettings(*this);
    }

    void
    clear(void)
    {
      fps = 0;
      quality = 0;
      reps = 0;
      tsize = 0;
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ImageTxSettings& other__ = static_cast<const ImageTxSettings&>(msg__);
      if (fps != other__.fps) return false;
      if (quality != other__.quality) return false;
      if (reps != other__.reps) return false;
      if (tsize != other__.tsize) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(fps, ptr__);
      ptr__ += IMC::serialize(quality, ptr__);
      ptr__ += IMC::serialize(reps, ptr__);
      ptr__ += IMC::serialize(tsize, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(fps, bfr__, size__);
      bfr__ += IMC::deserialize(quality, bfr__, size__);
      bfr__ += IMC::deserialize(reps, bfr__, size__);
      bfr__ += IMC::deserialize(tsize, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(fps, bfr__, size__);
      bfr__ += IMC::deserialize(quality, bfr__, size__);
      bfr__ += IMC::deserialize(reps, bfr__, size__);
      bfr__ += IMC::deserialize(tsize, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ImageTxSettings::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ImageTxSettings";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "fps", fps, nindent__);
      IMC::toJSON(os__, "quality", quality, nindent__);
      IMC::toJSON(os__, "reps", reps, nindent__);
      IMC::toJSON(os__, "tsize", tsize, nindent__);
    }
  };
}

#endif
