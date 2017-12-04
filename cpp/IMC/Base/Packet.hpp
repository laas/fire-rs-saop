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

#ifndef IMC_PACKET_HPP_INCLUDED_
#define IMC_PACKET_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <cstddef>
#include <ostream>

// IMC headers.
#include "Config.hpp"
#include "CRC16.hpp"
#include "ByteCopy.hpp"
#include "ByteBuffer.hpp"
#include "Message.hpp"
#include "Serialization.hpp"

namespace IMC
{
  class Packet
  {
  public:
    //! Serialize a message object.
    //! @param[in] msg message object.
    //! @param[out] bfr destination buffer.
    //! @param[in] size destination buffer size.
    //! @return number of bytes written to the destination buffer.
    static size_t
    serialize(const Message* msg, uint8_t* bfr, size_t size)
    {
      size_t total = msg->getSerializationSize();
      if (total > Message::maxSerializedSize())
        throw InvalidMessageSize(total);

      size_t n = total;
      uint8_t* ptr = bfr;

      if (size < n)
        throw BufferTooShort();

      ptr += serializeHeader(msg, bfr, size);
      msg->serializeFields(ptr);

      uint16_t crc = CRC16::compute(bfr, n - IMC_CONST_FOOTER_SIZE);
      IMC::serialize(crc, (bfr + (n - IMC_CONST_FOOTER_SIZE)));

      return n;
    }

    //! Serialize a message object.
    //! @param[in] msg message object.
    //! @param[out] bfr destination buffer.
    //! @return number of bytes written to the destination buffer.
    static size_t
    serialize(const Message* msg, ByteBuffer& bfr)
    {
      size_t size = msg->getSerializationSize();
      if (size > Message::maxSerializedSize())
        throw InvalidMessageSize(size);

      bfr.setSize(size);
      return serialize(msg, bfr.getBuffer(), size);
    }

    static Message*
    deserialize(const uint8_t* bfr,size_t bfr_len, Message* msg = NULL)
    {
      Header hdr;

      // Get the message header.
      deserializeHeader(hdr, bfr, bfr_len);

      // Check if we can unpack the message.
      if (hdr.size > bfr_len - (IMC_CONST_HEADER_SIZE + IMC_CONST_FOOTER_SIZE))
        throw BufferTooShort();

      return deserializePayload(hdr, bfr, bfr_len, msg);
    }

    static Message*
    deserialize(std::istream& ifs, ByteBuffer& bfr)
    {
      // Get the message header.
      bfr.setSize(IMC_CONST_HEADER_SIZE);
      ifs.read(bfr.getBufferSigned(), IMC_CONST_HEADER_SIZE);

      // If we're at the EOF there's nothing more to do.
      if (ifs.eof())
        return 0;

      if (ifs.gcount() < IMC_CONST_HEADER_SIZE)
        throw BufferTooShort();

      Header hdr;
      deserializeHeader(hdr, bfr.getBuffer(), IMC_CONST_HEADER_SIZE);

      // Get remaining data.
      std::streamsize remaining = hdr.size + IMC_CONST_FOOTER_SIZE;
      bfr.setSize(IMC_CONST_HEADER_SIZE + remaining);
      ifs.read(bfr.getBufferSigned() + IMC_CONST_HEADER_SIZE, remaining);

      if (ifs.gcount() < remaining)
        throw BufferTooShort();

      return deserializePayload(hdr, bfr.getBuffer(), IMC_CONST_HEADER_SIZE + remaining, 0);
    }

    static Message*
    deserialize(std::istream& ifs)
    {
      std::vector<char> data;

      // Get the message header.
      data.resize(IMC_CONST_HEADER_SIZE);
      ifs.read(&data[0], IMC_CONST_HEADER_SIZE);

      // If we're at the EOF there's nothing more to do.
      if (ifs.eof())
        return NULL;

      if (ifs.gcount() < IMC_CONST_HEADER_SIZE)
        throw BufferTooShort();

      Header hdr;
      deserializeHeader(hdr, (uint8_t*)&data[0], IMC_CONST_HEADER_SIZE);

      // Get remaining data.
      std::streamsize remaining = hdr.size + IMC_CONST_FOOTER_SIZE;
      data.resize(IMC_CONST_HEADER_SIZE + remaining);
      ifs.read(&data[IMC_CONST_HEADER_SIZE], remaining);

      if (ifs.gcount() < remaining)
        throw BufferTooShort();

      return deserializePayload(hdr, (uint8_t*)&data[0], IMC_CONST_HEADER_SIZE + remaining, 0);
    }

    static size_t
    serializeHeader(const Message* msg, uint8_t* bfr, size_t bfr_len)
    {
      (void)bfr_len;

      uint8_t* ptr = bfr;

      ptr += IMC::serialize((uint16_t)IMC_CONST_SYNC, ptr);
      ptr += IMC::serialize(msg->getId(), ptr);
      ptr += IMC::serialize((uint16_t)msg->getPayloadSerializationSize(), ptr);
      ptr += IMC::serialize(msg->getTimeStamp(), ptr);
      ptr += IMC::serialize((uint16_t)msg->getSource(), ptr);
      ptr += IMC::serialize(msg->getSourceEntity(), ptr);
      ptr += IMC::serialize((uint16_t)msg->getDestination(), ptr);
      ptr += IMC::serialize(msg->getDestinationEntity(), ptr);

      return ptr - bfr;
    }

    static void
    deserializeHeader(Header& hdr, const uint8_t* bfr, size_t bfr_len)
    {
      // Check if we can at least parse the header.
      if (bfr_len < IMC_CONST_HEADER_SIZE)
        throw BufferTooShort();

      // Parse synchronize number and get byte order.
      ByteCopy::copy(hdr.sync, bfr);

      // Read header.
      if (hdr.sync == IMC_CONST_SYNC)
      {
        ByteCopy::copy(hdr.mgid, bfr + 2);
        ByteCopy::copy(hdr.size, bfr + 4);
        ByteCopy::copy(hdr.timestamp, bfr + 6);
        ByteCopy::copy(hdr.src, bfr + 14);
        ByteCopy::copy(hdr.src_ent, bfr + 16);
        ByteCopy::copy(hdr.dst, bfr + 17);
        ByteCopy::copy(hdr.dst_ent, bfr + 19);
      }
      else if (hdr.sync == IMC_CONST_SYNC_REV)
      {
        ByteCopy::rcopy(hdr.mgid, bfr + 2);
        ByteCopy::rcopy(hdr.size, bfr + 4);
        ByteCopy::rcopy(hdr.timestamp, bfr + 6);
        ByteCopy::rcopy(hdr.src, bfr + 14);
        ByteCopy::rcopy(hdr.src_ent, bfr + 16);
        ByteCopy::rcopy(hdr.dst, bfr + 17);
        ByteCopy::rcopy(hdr.dst_ent, bfr + 19);
      }
      else
      {
        throw InvalidSync(hdr.sync);
      }
    }

    static Message*
    deserializePayload(const Header& hdr, const uint8_t* bfr, size_t bfr_len, Message* msg)
    {
      (void)bfr_len;

      // Retrieve CRC
      uint16_t rcrc = 0;

      if (hdr.sync == IMC_CONST_SYNC_REV)
        ByteCopy::rcopy(rcrc, bfr + IMC_CONST_HEADER_SIZE + hdr.size);
      else
        ByteCopy::copy(rcrc, bfr + IMC_CONST_HEADER_SIZE + hdr.size);

      // Validate CRC.
      uint16_t crc = CRC16::compute(bfr, IMC_CONST_HEADER_SIZE + hdr.size);

      if (crc != rcrc)
        throw InvalidCrc();

      // Produce a message of the given type.
      if (msg == NULL)
      {
        msg = Factory::produce(hdr.mgid);
        if (msg == 0)
          throw InvalidMessageId(hdr.mgid);
      }
      else
      {
        if (msg->getId() != hdr.mgid)
          throw InvalidMessageId(hdr.mgid);
      }

      // Deserialize message fields.
      try
      {
        if (hdr.sync == IMC_CONST_SYNC_REV)
          msg->reverseDeserializeFields(bfr + IMC_CONST_HEADER_SIZE, hdr.size);
        else
          msg->deserializeFields(bfr + IMC_CONST_HEADER_SIZE, hdr.size);
      }
      catch (...)
      {
        delete msg;
        throw;
      }

      msg->setTimeStamp(hdr.timestamp);
      msg->setSource(hdr.src);
      msg->setSourceEntity(hdr.src_ent);
      msg->setDestination(hdr.dst);
      msg->setDestinationEntity(hdr.dst_ent);

      return msg;
    }
  };
}

#endif
