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

#ifndef PLANNING_CPP_SAOP_NEPTUS_H
#define PLANNING_CPP_SAOP_NEPTUS_H

#include <vector>
#include <cstdlib>
#include <iostream>
#include <string>
#include <array>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../../IMC/Base/ByteBuffer.hpp"
#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Base/Message.hpp"


//#include "../ext/coordinates.hpp"
//#include "../core/waypoint.hpp"
//#include "../neptus/imc_message_factories.hpp"
//#include "../vns/plan.hpp"


namespace SAOP {

    namespace neptus {
        using boost::asio::ip::tcp;

        class Server {
        private:
            unsigned short port;

            size_t capacity = 1024;
        public:

            explicit Server(unsigned short port) : port(port) {}

            void run() {
                for (;;) {
                    boost::asio::io_service io_service;
                    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
                    std::shared_ptr<tcp::socket> sock(new tcp::socket(io_service));
                    std::cout << "waiting to accept connection" << std::endl;
                    a.accept(*sock);
                    boost::thread t(boost::bind(&Server::session, this, sock));
                }
            }

            void session(std::shared_ptr<tcp::socket> sock) {
                try {
                    for (;;) {
                        IMC::ByteBuffer bb = IMC::ByteBuffer();
                        bb.setSize(2048);

                        boost::system::error_code error;
                        size_t length = sock->read_some(boost::asio::buffer(bb.getBuffer(), bb.getSize()), error);
                        if (error == boost::asio::error::eof)
                            break; // Connection closed cleanly by peer.
                        else if (error)
                            throw boost::system::system_error(error); // Some other error.



                        // Deserialize IMC packet
                        auto hb = IMC::Packet::deserialize(bb.getBuffer(), bb.getSize());
                        // Can I cast a Message to its message type? Heartbeat for instance

                        std::cout << "Received: " << hb->toString() << std::endl;



//                        boost::asio::write(*sock, boost::asio::buffer(data, length));
                    }
                }
                catch (std::exception& e) {
                    std::cerr << "Exception in thread: " << e.what() << "\n";
                }
            }
        };
    }
}

int main() {
    SAOP::neptus::Server s(8888);
    s.run();
    return 0;
}


#endif //PLANNING_CPP_SAOP_NEPTUS_H
