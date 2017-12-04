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

// ISO C++ 98 headers.
#include <iostream>
#include <iomanip>
#include <cstdio>
#include <map>

// IMC Base headers.
#include "String.hpp"
#include "Exceptions.hpp"
#include "Factory.hpp"

// IMC API headers.
#include "../Spec/AllMessages.hpp"

//! Construct a std::map statically from an array of std::pairs.
#define IMC_DECLARE_STATIC_MAP(name, ta, tb, ps)                    \
  static std::map<ta, tb> name(ps, (ps) + sizeof(ps) / sizeof((ps)[0]))

namespace IMC
{
  typedef Message* (*Creator) (void);

  template <typename Type>
  static Message*
  create(void)
  {
    return new Type();
  }

  static std::pair<uint32_t, std::string> pairs_id_abbrev[] =
  {
#define MESSAGE(id, abbrev, md5)                    \
    std::pair<uint32_t, std::string>(id, #abbrev),
#include "../Spec/Factory.xdef"
  };

  static std::pair<std::string, uint32_t> pairs_abbrev_id[] =
  {
#define MESSAGE(id, abbrev, md5)                    \
    std::pair<std::string, uint32_t>(#abbrev, id),
#include "../Spec/Factory.xdef"
  };

  static std::pair<int, Creator> creator_pairs_id[] =
  {
#define MESSAGE(id, abbrev, md5)                    \
    std::pair<int, Creator>(id, &create<abbrev>),
#include "../Spec/Factory.xdef"
  };

  IMC_DECLARE_STATIC_MAP(creators_by_id, int, Creator, creator_pairs_id);
  IMC_DECLARE_STATIC_MAP(map_id_abbrev, uint32_t, std::string, pairs_id_abbrev);
  IMC_DECLARE_STATIC_MAP(map_abbrev_id, std::string, uint32_t, pairs_abbrev_id);

  Message*
  Factory::produce(uint32_t id)
  {
    if (creators_by_id[id])
      return creators_by_id[id]();

    return 0;
  }

  Message*
  Factory::produce(const std::string& name)
  {
    uint32_t id = getIdFromAbbrev(name);

    return produce(id);
  }

  std::string
  Factory::getAbbrevFromId(uint32_t id)
  {
    std::map<uint32_t, std::string>::iterator itr = map_id_abbrev.find(id);

    if (itr == map_id_abbrev.end())
      throw InvalidMessageId(id);

    return itr->second;
  }

  uint32_t
  Factory::getIdFromAbbrev(const std::string& name)
  {
    std::map<std::string, uint32_t>::iterator itr = map_abbrev_id.find(name);

    if (itr == map_abbrev_id.end())
      throw InvalidMessageAbbrev(name);

    return itr->second;
  }

  void
  Factory::getAbbrevs(std::vector<std::string>& v)
  {
    for(std::map<std::string, uint32_t>::iterator itr = map_abbrev_id.begin();
        itr != map_abbrev_id.end(); itr++)
    {
      v.push_back(itr->first);
    }
  }

  void
  Factory::getIds(std::vector<uint32_t>& v)
  {
    for(std::map<std::string, uint32_t>::iterator itr = map_abbrev_id.begin();
        itr != map_abbrev_id.end(); itr++)
    {
      v.push_back(itr->second);
    }
  }
}
