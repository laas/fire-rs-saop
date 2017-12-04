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
// Automatically generated.                                                 *
//***************************************************************************
// IMC XML MD5: 4d8734a1111656aac56f803acdc90c22                            *
//***************************************************************************

#ifndef IMC_REPORTCONTROL_HPP_INCLUDED_
#define IMC_REPORTCONTROL_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <ostream>
#include <string>
#include <vector>

// IMC headers.
#include "../Base/Config.hpp"
#include "../Base/Message.hpp"
#include "../Base/InlineMessage.hpp"
#include "../Base/MessageList.hpp"
#include "../Base/JSON.hpp"
#include "../Base/Serialization.hpp"
#include "../Spec/Enumerations.hpp"
#include "../Spec/Bitfields.hpp"

namespace IMC
{
  //! Report Control.
  class ReportControl: public Message
  {
  public:
    //! Operation.
    enum OperationEnum
    {
      //! Request Start of Reports.
      OP_REQUEST_START = 0,
      //! Report Started.
      OP_STARTED = 1,
      //! Request Stop of Reports.
      OP_REQUEST_STOP = 2,
      //! Report Stopped.
      OP_STOPPED = 3,
      //! Request Single Reports.
      OP_REQUEST_REPORT = 4,
      //! Single Report Sent.
      OP_REPORT_SENT = 5
    };

    //! Communication Interface.
    enum CommunicationInterfaceBits
    {
      //! Acoustic.
      CI_ACOUSTIC = 0x01,
      //! Satellite.
      CI_SATELLITE = 0x02,
      //! GSM.
      CI_GSM = 0x04,
      //! Mobile.
      CI_MOBILE = 0x08
    };

    //! Operation.
    uint8_t op;
    //! Communication Interface.
    uint8_t comm_interface;
    //! Period.
    uint16_t period;
    //! Destination System.
    std::string sys_dst;

    static uint16_t
    getIdStatic(void)
    {
      return 513;
    }

    static ReportControl*
    cast(Message* msg__)
    {
      return (ReportControl*)msg__;
    }

    ReportControl(void)
    {
      m_header.mgid = ReportControl::getIdStatic();
      clear();
    }

    ReportControl*
    clone(void) const
    {
      return new ReportControl(*this);
    }

    void
    clear(void)
    {
      op = 0;
      comm_interface = 0;
      period = 0;
      sys_dst.clear();
    }

    bool
    fieldsEqual(const Message& msg__) const
    {
      const IMC::ReportControl& other__ = static_cast<const ReportControl&>(msg__);
      if (op != other__.op) return false;
      if (comm_interface != other__.comm_interface) return false;
      if (period != other__.period) return false;
      if (sys_dst != other__.sys_dst) return false;
      return true;
    }

    uint8_t*
    serializeFields(uint8_t* bfr__) const
    {
      uint8_t* ptr__ = bfr__;
      ptr__ += IMC::serialize(op, ptr__);
      ptr__ += IMC::serialize(comm_interface, ptr__);
      ptr__ += IMC::serialize(period, ptr__);
      ptr__ += IMC::serialize(sys_dst, ptr__);
      return ptr__;
    }

    size_t
    deserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(comm_interface, bfr__, size__);
      bfr__ += IMC::deserialize(period, bfr__, size__);
      bfr__ += IMC::deserialize(sys_dst, bfr__, size__);
      return bfr__ - start__;
    }

    size_t
    reverseDeserializeFields(const uint8_t* bfr__, size_t size__)
    {
      const uint8_t* start__ = bfr__;
      bfr__ += IMC::deserialize(op, bfr__, size__);
      bfr__ += IMC::deserialize(comm_interface, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(period, bfr__, size__);
      bfr__ += IMC::reverseDeserialize(sys_dst, bfr__, size__);
      return bfr__ - start__;
    }

    uint16_t
    getId(void) const
    {
      return ReportControl::getIdStatic();
    }

    const char*
    getName(void) const
    {
      return "ReportControl";
    }

    size_t
    getFixedSerializationSize(void) const
    {
      return 4;
    }

    size_t
    getVariableSerializationSize(void) const
    {
      return IMC::getSerializationSize(sys_dst);
    }

    void
    fieldsToJSON(std::ostream& os__, unsigned nindent__) const
    {
      IMC::toJSON(os__, "op", op, nindent__);
      IMC::toJSON(os__, "comm_interface", comm_interface, nindent__);
      IMC::toJSON(os__, "period", period, nindent__);
      IMC::toJSON(os__, "sys_dst", sys_dst, nindent__);
    }
  };
}

#endif
