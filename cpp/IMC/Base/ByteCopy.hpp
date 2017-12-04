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

#ifndef IMC_BYTE_COPY_HPP_INCLUDED_
#define IMC_BYTE_COPY_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <iostream>
#include <cstring>

// IMC Base headers.
#include "Config.hpp"

namespace IMC
{
  class ByteCopy
  {
  public:
    //! Copy one byte of a memory area.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    copy1b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[0];
      return 1;
    }

    //! Copy one byte of a memory area.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    rcopy1b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[0];
      return 1;
    }

    static inline size_t
    copy(uint8_t& dst, const uint8_t* src)
    {
      return copy1b(&dst, src);
    }

    static inline size_t
    rcopy(uint8_t& dst, const uint8_t* src)
    {
      return rcopy1b(&dst, src);
    }

    //! Copy two bytes of a memory area.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    copy2b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[0];
      dst[1] = src[1];
      return 2;
    }

    //! Copy two bytes of a memory area while reversing the order of
    //! the bytes.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    rcopy2b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[1];
      dst[1] = src[0];
      return 2;
    }

    static inline size_t
    copy(uint16_t& dst, const uint8_t* src)
    {
      return copy2b(reinterpret_cast<uint8_t*>(&dst), src);
    }

    static inline size_t
    rcopy(int16_t& dst, const uint8_t* src)
    {
      return rcopy2b((uint8_t*)&dst, src);
    }

    static inline size_t
    rcopy(uint16_t& dst, const uint8_t* src)
    {
      return rcopy2b(reinterpret_cast<uint8_t*>(&dst), src);
    }

    //! Copy four bytes of a memory area while reversing the order of
    //! the bytes.
    //! @param dst destination memory area.
    //! @param src source memory area.
    static inline size_t
    rcopy4b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[3];
      dst[1] = src[2];
      dst[2] = src[1];
      dst[3] = src[0];
      return 4;
    }

    static inline size_t
    rcopy(int32_t& dst, const uint8_t* src)
    {
      return rcopy4b((uint8_t*)&dst, src);
    }

    static inline size_t
    rcopy(uint32_t& dst, const uint8_t* src)
    {
      return rcopy4b((uint8_t*)&dst, src);
    }

    static inline size_t
    rcopy(float& dst, const uint8_t* src)
    {
      return rcopy4b((uint8_t*)&dst, src);
    }

    //! Copy eight bytes of a memory area.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    copy8b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[0];
      dst[1] = src[1];
      dst[2] = src[2];
      dst[3] = src[3];
      dst[4] = src[4];
      dst[5] = src[5];
      dst[6] = src[6];
      dst[7] = src[7];
      return 8;
    }

    //! Copy eight bytes of a memory area while reversing the order of
    //! the bytes.
    //! @param dst destination memory area.
    //! @param src source memory area.
    //! @return number of copied bytes.
    static inline size_t
    rcopy8b(uint8_t* dst, const uint8_t* src)
    {
      dst[0] = src[7];
      dst[1] = src[6];
      dst[2] = src[5];
      dst[3] = src[4];
      dst[4] = src[3];
      dst[5] = src[2];
      dst[6] = src[1];
      dst[7] = src[0];
      return 8;
    }

    static inline size_t
    rcopy(int64_t& dst, const uint8_t* src)
    {
      return rcopy8b(reinterpret_cast<uint8_t*>(&dst), src);
    }

    static inline size_t
    copy(double& dst, const uint8_t* src)
    {
      return copy8b(reinterpret_cast<uint8_t*>(&dst), src);
    }

    static inline size_t
    rcopy(double& dst, const uint8_t* src)
    {
      return rcopy8b(reinterpret_cast<uint8_t*>(&dst), src);
    }
  };
}

#endif
