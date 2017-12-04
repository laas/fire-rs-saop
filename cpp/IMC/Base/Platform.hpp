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

#ifndef IMC_PLATFORM_HPP_INCLUDED_
#define IMC_PLATFORM_HPP_INCLUDED_

// Probe operating system.
#if defined(__linux__)
#  define IMC_OS_LINUX   1
#  define IMC_OS_POSIX   1
#  define IMC_OS_NAME    "Linux"

#elif defined(_WIN32)
#  define IMC_OS_WINDOWS 1
#  define IMC_OS_NAME    "Windows"

#elif defined(__APPLE__)
#  define IMC_OS_MACOS   1
#  define IMC_OS_POSIX   1
#  define IMC_OS_NAME    "macOS"

#elif defined(__APPLE__)
#  define IMC_OS_FREEBSD 1
#  define IMC_OS_POSIX   1
#  define IMC_OS_NAME    "FreeBSD"

#else
#  error unknown operating system
#endif

// Architecture.
#if defined(__amd64__) || defined(__amd64) || defined(_AMD64_) || defined(__x86_64__) || defined(__x86_64__) || defined(__x86_64) || defined(_M_X64)
#  define IMC_CPU_AMD64         1
#  define IMC_CPU_LE            1
#  define IMC_CPU_64B           1
#  define IMC_CPU_NAME          "amd64"

#elif defined(__i386__) || defined(__i386) || defined(_M_IX86) || defined(__x86_32__)
#  define IMC_CPU_X86           1
#  define IMC_CPU_LE            1
#  define IMC_CPU_32B           1
#  define IMC_CPU_NAME          "x86"

#elif defined(__arm__)
#  define IMC_CPU_ARM           1
#  define IMC_CPU_LE            1
#  define IMC_CPU_32B           1
#  define IMC_CPU_NAME          "arm"

#else
#  error unknown architecture

#endif

// Compiler.
#if defined(__clang__)
#  define IMC_CXX_CLANG         1
#  define IMC_CXX_NAME          "clang"

#elif defined(__GNUC__)
#  define IMC_CXX_GNU           1
#  define IMC_CXX_NAME           "gcc"

#elif defined(_MSC_VER)
#  define IMC_CXX_MSC           1
#  define IMC_CXX_NAME          "msc"


#else
#  error unknown compiler

#endif

#endif
