//***************************************************************************
// Copyright 2007-2016 Universidade do Porto - Faculdade de Engenharia      *
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
// Author: Eduardo Marques                                                  *
//***************************************************************************

#ifndef IMC_PARSER_HPP_INCLUDED_
#define IMC_PARSER_HPP_INCLUDED_

// ISO C++ headers.
#include <vector>
#include <queue>
#include <set>
#include <memory>

// IMC headers.
#include "Message.hpp"
#include "Packet.hpp"

namespace IMC
{
  //! Parser class.
  class Parser
  {
  public:
    Parser()
    {
      reset();
    }

    void
    reset()
    {
      m_stage = PS_SYNC;
      m_pos = 0;
      m_buf.clear();
    }

    //! Parse byte and return message if parsing of one message is done.
    //! @param byte data byte
    //! @return defined message or 0
    Message*
    parse(uint8_t byte)
    {
      Message* m = 0;
      m_buf.push_back(byte);

      while (true)
      {
        size_t n = m_buf.size() - m_pos;

        if (n == 0)
        {
          reset(); // discard unneeded data
          break;
        }

        if (m_stage == PS_SYNC)
        {
          // This would not be strictly necessary but is more efficient
          // than handling recurrent exceptions from Packet::deserializeHeader
          if (n == 1)
            break; // need more data

          uint16_t sync = (m_buf[m_pos] << 8) | m_buf[m_pos + 1];

          if (sync != IMC_CONST_SYNC && sync != IMC_CONST_SYNC_REV)
          {
            m_pos++; // invalid sync, advance
            continue;
          }
          m_stage = PS_HEADER; // sync is ok, get rest of header
        }

        if (m_stage == PS_HEADER)
        {
          if (n < IMC_CONST_HEADER_SIZE)
            break;  // need more data

          Packet::deserializeHeader(m_header, &m_buf[m_pos], n);
          m_stage = PS_PAYLOAD; // done with header
        }

        // on to c_payload stage
        if (n < (size_t)(m_header.size + IMC_CONST_HEADER_SIZE + IMC_CONST_FOOTER_SIZE))
          break;  // need more data

        // all payload data available
        m_stage = PS_SYNC; // the next stage in any case

        try
        {
          m = Packet::deserializePayload(m_header, &m_buf[m_pos], n, 0);
        }
        catch (...)
        {
          ++m_pos; // try to find sync again from current position
          continue;
        }

        m_pos += n;

        if (m_pos == m_buf.size())
          reset();  // discard unneeded data

        break;
      }

      return m;
    }

  private:
    //! Parser stage constants.
    enum ParserStage
    {
      PS_SYNC,
      PS_HEADER,
      PS_PAYLOAD
    };

    //! Parser stage.
    ParserStage m_stage;
    //! Internal buffer.
    std::vector<uint8_t> m_buf;
    //! Buffer position.
    unsigned int m_pos;
    //! Holds parsed header (c_payload stage).
    Header m_header;
  };
}

#endif
