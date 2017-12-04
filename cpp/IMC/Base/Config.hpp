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

#ifndef IMC_CONFIG_HPP
#define IMC_CONFIG_HPP

// IMC Base headers.
#include "Platform.hpp"

#if defined(IMC_OS_POSIX)
#  include <stdint.h>
#elif defined(IMC_OS_WINDOWS)
#  include <windows.h>
typedef INT8 int8_t;
typedef UINT8 uint8_t;
typedef INT16 int16_t;
typedef UINT16 uint16_t;
typedef INT32 int32_t;
typedef UINT32 uint32_t;
typedef INT64 int64_t;
typedef UINT64 uint64_t;

#else
#  error failed to find standard integer definition
#endif

// Declaration for exporting symbols.
#if defined(IMC_OS_WINDOWS)
#  if defined(IMC_DLL_EXPORT)
#    define IMC_SYM_EXPORT __declspec(dllexport)
#  else
#    define IMC_SYM_EXPORT __declspec(dllimport)
#  endif

#else
#  define IMC_SYM_EXPORT
#endif

#endif
