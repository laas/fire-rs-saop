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

#ifndef IMC_FACTORY_HPP_INCLUDED_
#define IMC_FACTORY_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <string>
#include <vector>

// IMC Base headers.
#include "Config.hpp"
#include "Message.hpp"

namespace IMC
{
  class IMC_SYM_EXPORT Factory;

  class Factory
  {
  public:
    //! Produce a message object by identification number.
    //! @param key message identification number.
    //! @return message object allocated on the heap.
    static Message*
    produce(uint32_t key);

    //! Produce a message object by name.
    //! @param name message name.
    //! @return message object allocated on the heap.
    static Message*
    produce(const std::string& name);

    //! Retrieve all message abbreviations.
    //! @param v output vector
    static void
    getAbbrevs(std::vector<std::string>& v);

    //! Retrieve all message identification numbers.
    //! @param v output vector
    static void
    getIds(std::vector<uint32_t>& v);

    //! Retrieve the corresponding message abbreviation from the
    //! identification number.
    //! @param id identification number.
    //! @return abbreviation.
    static std::string
    getAbbrevFromId(uint32_t id);

    //! Retrieve the corresponding message identification number from the
    //! abbreviation.
    //! @param name abbreviation.
    //! @return identification number.
    static uint32_t
    getIdFromAbbrev(const std::string& name);
  };
}

#endif
