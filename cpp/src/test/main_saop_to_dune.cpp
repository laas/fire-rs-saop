/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

using boost::asio::ip::udp;

#include "../neptus/imc_message_factories.hpp"


enum { max_length = 1024 };

int main(int argc, char* argv[])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: blocking_udp_echo_client <host> <port>\n";
      return 1;
    }

    boost::asio::io_service io_service;

    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), argv[1], argv[2]);
    udp::resolver::iterator iterator = resolver.resolve(query);

    using namespace std; // For strlen.
//    std::cout << "Enter message: ";
//    char request[max_length];
//    std::cin.getline(request, max_length);
//    size_t request_length = strlen(request);
    auto pk = IMC::Packet();
    IMC::ByteBuffer bb = IMC::ByteBuffer();
    bb.setSize(max_length);
    auto hbb = SAOP::neptus::PlanDBFactory::make_message();
    hbb.setSource(3088);
    hbb.setSourceEntity(25);
    hbb.setDestination(24290);
    hbb.setDestinationEntity(8);
    hbb.setTimeStamp();
    pk.serialize(&hbb, bb);

    std::cout << "Sent: " << hbb.toString() << std::endl;

    s.send_to(boost::asio::buffer(bb.getBuffer(), bb.getSize()), *iterator);
//
//    char reply[max_length];
//    udp::endpoint sender_endpoint;
//    size_t reply_length = s.receive_from(
//        boost::asio::buffer(reply, max_length), sender_endpoint);
//    std::cout << "Reply is: ";
//    std::cout.write(reply, reply_length);
//    std::cout << "\n";
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}