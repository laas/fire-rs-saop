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

#ifndef IMC_STRING_HPP_INCLUDED_
#define IMC_STRING_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <cstdarg>

// IMC headers.
#include "../Base/Config.hpp"

namespace IMC
{
  class String
  {
  public:
    static std::string
    toHex(const std::vector<char>& str)
    {
      char bfr[3];
      std::string result;

      for (unsigned int i = 0; i < str.size(); ++i)
      {
        format(bfr, 3, "%02X", (uint8_t)str[i]);
        result.push_back(bfr[0]);
        result.push_back(bfr[1]);
      }

      return result;
    }

    static std::string
    toHex(unsigned nr)
    {
      std::stringstream ss;
      ss << std::hex << nr;
      return ss.str();
    }

    template <typename Type>
    static std::string
    str(Type t)
    {
      std::stringstream ss;
      ss << t;
      return ss.str();
    }

    static std::string
    str(const char* fmt, ...)
    {
      std::va_list ap;
      va_start(ap, fmt);
      std::string result = strVl(fmt, ap);
      va_end(ap);
      return result;
    }

    static std::string
    strVl(const char* fmt, std::va_list ap)
    {
      char bfr[1024] = {0};
      formatVl(bfr, sizeof(bfr), fmt, ap);
      return std::string(bfr);
    }

    static int
    format(char* str, size_t size, const char* fmt, ...)
    {
      std::va_list ap;
      va_start(ap, fmt);
      int rv = formatVl(str, size, fmt, ap);
      va_end(ap);
      return rv;
    }

    static int
    formatVl(char* str, size_t size, const char* fmt, std::va_list ap)
    {
#if defined(IMC_OS_POSIX)
      int rv = vsnprintf(str, size, fmt, ap);
#elif defined(IMC_OS_WINDOWS)
      int rv = vsnprintf_s(str, size, size - 1, fmt, ap);
#else
      int rv = std::vsprintf(str, fmt, ap);
#endif
      return rv;
    }
  };
}

#endif
