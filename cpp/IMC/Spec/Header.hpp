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

#ifndef IMC_HEADER_HPP_INCLUDED_
#define IMC_HEADER_HPP_INCLUDED_

// IMC headers.
#include "../Base/Config.hpp"

namespace IMC
{
  //! Header format.
  struct Header
  {
    //! Synchronization Number.
    uint16_t sync;
    //! Message Identification Number.
    uint16_t mgid;
    //! Message size.
    uint16_t size;
    //! Time stamp.
    double timestamp;
    //! Source Address.
    uint16_t src;
    //! Source Entity.
    uint8_t src_ent;
    //! Destination Address.
    uint16_t dst;
    //! Destination Entity.
    uint8_t dst_ent;
  };
}

#endif
