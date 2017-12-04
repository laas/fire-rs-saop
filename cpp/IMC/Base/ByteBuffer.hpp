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

#ifndef IMC_BYTE_BUFFER_HPP_INCLUDED_
#define IMC_BYTE_BUFFER_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <cstdlib>
#include <ostream>
#include <cstring>

// IMC Base headers.
#include "Config.hpp"

namespace IMC
{
  class ByteBuffer
  {
    friend std::ostream&
    operator<<(std::ostream& os, ByteBuffer& buffer);

  public:
    explicit
    ByteBuffer(size_t capacity = 128):
      m_buffer(NULL),
      m_capacity(ByteBuffer::computeNextPowerOfTwo(capacity)),
      m_size(0)
    {
      m_buffer = (uint8_t*)std::malloc(m_capacity);
    }

    inline
    ~ByteBuffer()
    {
      std::free(m_buffer);
    }

    inline size_t
    getCapacity() const
    {
      return m_capacity;
    }

    inline void
    grow(size_t size)
    {
      if (size > m_capacity)
      {
        m_capacity = computeNextPowerOfTwo(size);
        m_buffer = (uint8_t*)std::realloc(m_buffer, m_capacity);
      }
    }

    inline void
    setSize(size_t size)
    {
      grow(size);
      m_size = size;
    }

    inline size_t
    getSize()
    {
      return m_size;
    }

    inline uint8_t*
    getBuffer()
    {
      return m_buffer;
    }

    inline char*
    getBufferSigned()
    {
      return (char*)m_buffer;
    }

    void
    append(const uint8_t* data, size_t size)
    {
      grow(m_size + size);
      std::memcpy(m_buffer + m_size, data, size);
      m_size += size;
    }

    void
    appendSigned(const char* data, size_t size)
    {
      append((uint8_t*)data, size);
    }

   private:
    //! Internal buffer.
    uint8_t* m_buffer;
    //! Internal buffer's capacity.
    size_t m_capacity;
    //! Internal buffer's used space.
    size_t m_size;

    //! Compute the next power of two of a value.
    //! @param val value.
    //! @return next power of two.
    static inline size_t
    computeNextPowerOfTwo(size_t val)
    {
      size_t r = 1;

      while (r < val)
        r <<= 1;

      return r;
    }
  };

  inline std::ostream&
  operator<<(std::ostream& os, ByteBuffer& buffer)
  {
    os.write((char*)buffer.m_buffer, buffer.m_size);
    return os;
  }
}

#endif
