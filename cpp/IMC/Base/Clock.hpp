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

#ifndef IMC_CLOCK_HPP_INCLUDED_
#define IMC_CLOCK_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ctime>
#include <cstring>
#include <cerrno>

// IMC Base headers.
#include "Config.hpp"

// Platform specific headers.
#if defined(IMC_OS_WINDOWS)
#  include <windows.h>
#elif defined(IMC_OS_POSIX)
#  include <pthread.h>
#  include <sys/time.h>
#endif

namespace IMC
{
  //! Number of nanoseconds in a second.
  static const unsigned c_nsec_per_sec = 1000000000u;
    //! Number of nanoseconds in a second (floating point).
  static const double c_nsec_per_sec_fp = 1000000000.0;

  //! %System clock routines.
  class Clock
  {
  public:
    //! Get the amount of time (in nanoseconds) elapsed since the
    //! UNIX Epoch (Midnight UTC of January 1, 1970).
    //! @return time in nanoseconds.
    static uint64_t
    getSinceEpochNsec()
    {
      // POSIX RT.
#if defined(CLOCK_REALTIME)
      timespec ts = {0, 0};
      clock_gettime(CLOCK_REALTIME, &ts);
      return (uint64_t)ts.tv_sec * c_nsec_per_sec + (uint64_t)ts.tv_nsec;

      // POSIX.
#elif defined(IMC_OS_POSIX)
      timeval tv;
      gettimeofday(&tv, 0);
      return (uint64_t)tv.tv_sec * c_nsec_per_sec + (uint64_t)tv.tv_usec * 1000;

      // Microsoft Windows.
#elif defined(IMC_OS_WINDOWS)
      FILETIME ft;
      uint64_t tm;
      GetSystemTimeAsFileTime(&ft);
      std::memcpy(&tm, &ft, sizeof(uint64_t));

      // Subtract number of 100-nanosecond intervals between the beginning of the Windows
      // epoch (Jan. 1, 1601) and the Unix epoch (Jan. 1, 1970).
      tm -= 116444736000000000ULL;

      return tm * 100;

      // Unsupported system.
#else
#  error Clock::getSinceEpochNsec() is not yet implemented in this system.

#endif
    }

    //! Get the amount of time (in seconds) elapsed since the
    //! UNIX Epoch (Midnight UTC of January 1, 1970).
    //! @return time in seconds.
    static double
    getSinceEpoch()
    {
      return getSinceEpochNsec() / c_nsec_per_sec_fp;
    }
  };
}

#endif
