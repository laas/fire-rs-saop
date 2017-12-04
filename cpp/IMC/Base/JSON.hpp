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

#ifndef IMC_JSON_HPP_INCLUDED_
#define IMC_JSON_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <vector>
#include <string>

// IMC Base headers.
#include "Config.hpp"
#include "String.hpp"

namespace IMC
{
  // Indentation levels.
  static const char* c_imc_json_levels[] =
  {
    "",
    " ",
    "  ",
    "   ",
    "    ",
    "     ",
    "      ",
    "       ",
    "        ",
    "         ",
    "          ",
    "           ",
    "            "
  };

  // Number of indentation levels.
  static const unsigned c_imc_levels_count = sizeof(c_imc_json_levels) / sizeof(const char*);

  //! Return a string with a given number of white spaces.
  //! @param[in] count number of white spaces.
  //! @return string with 'count' white spaces.
  static inline const char*
  indentJSON(unsigned count)
  {
    if (count > c_imc_levels_count)
      count = c_imc_levels_count;

    return c_imc_json_levels[count];
  }

  //! Convert a generic type to a JSON string.
  //! @param[in] os output stream.
  //! @param[in] label label.
  //! @param[in] value value.
  //! @param[in] nindent number of indentation spaces.
  //! @param[in] prefix prefix character.
  template <typename Type>
  inline void
  toJSON(std::ostream& os, const char* label, const Type& value, unsigned nindent, char prefix = ',')
  {
    const char* indent = indentJSON(nindent);
    os << prefix << '\n' << indent << '"' << label << "\": \"" << value << '"';
  }

  //! Convert an 8 bit signed integer type to a JSON string.
  //! @param[in] os output stream.
  //! @param[in] label label.
  //! @param[in] value value.
  //! @param[in] nindent number of indentation spaces.
  //! @param[in] prefix prefix character.
  template <>
  inline void
  toJSON(std::ostream& os, const char* label, const int8_t& value, unsigned nindent, char prefix)
  {
    const char* indent = indentJSON(nindent);
    os << prefix << '\n' << indent << '"' << label << "\": \"" << (int)value << '"';
  }

  //! Convert an 8 bit unsigned integer type to a JSON string.
  //! @param[in] os output stream.
  //! @param[in] label label.
  //! @param[in] value value.
  //! @param[in] nindent number of indentation spaces.
  //! @param[in] prefix prefix character.
  template <>
  inline void
  toJSON(std::ostream& os, const char* label, const uint8_t& value, unsigned nindent, char prefix)
  {
    const char* indent = indentJSON(nindent);
    os << prefix << '\n' << indent << '"' << label << "\": \"" << (unsigned)value << '"';
  }

  //! Convert a string to a JSON string.
  //! @param[in] os output stream.
  //! @param[in] label label.
  //! @param[in] value value.
  //! @param[in] nindent number of indentation spaces.
  //! @param[in] prefix prefix character.
  template <>
  inline void
  toJSON(std::ostream& os, const char* label, const std::string& value, unsigned nindent, char prefix)
  {
    const char* indent = indentJSON(nindent);
    os << prefix << '\n' << indent << '"' << label << "\": \"" << value << '"';
  }

  //! Convert a character vector type to a JSON string.
  //! @param[in] os output stream.
  //! @param[in] label label.
  //! @param[in] value value.
  //! @param[in] nindent number of indentation spaces.
  //! @param[in] prefix prefix character.
  template <>
  inline void
  toJSON(std::ostream& os, const char* label, const std::vector<char>& value, unsigned nindent, char prefix)
  {
    const char* indent = indentJSON(nindent);
    os << prefix << '\n' << indent << '"' << label << "\": \"" << String::toHex(value) << '"';
  }
}

#endif
