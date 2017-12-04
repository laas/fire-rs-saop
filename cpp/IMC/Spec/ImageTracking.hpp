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

#ifndef IMC_IMAGETRACKING_HPP_INCLUDED_
#define IMC_IMAGETRACKING_HPP_INCLUDED_

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
#include "../Spec/Maneuver.hpp"

namespace IMC
{
  //! Image Tracking.
  class ImageTracking: public Maneuver
  {
  public:

    static uint16_t
    getIdStatic(void)
    {
      return 490;
    }

    static ImageTracking*
    cast(Message* msg__)
    {
      return (ImageTracking*)msg__;
    }

    ImageTracking(void)
    {
      m_header.mgid = ImageTracking::getIdStatic();
      clear();
    }

    ImageTracking*
    clone(void) const
    {
      return new ImageTracking(*this);
    }

    void
    clear(void)
    {
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      return bfr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      (void)bfr__;
      (void)size__;
      return 0;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      (void)bfr__;
      (void)size__;
      return 0;
    }

    uint16_t
    getId(void) const
    {
      return ImageTracking::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ImageTracking";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 0;
    }
  };
}

#endif
