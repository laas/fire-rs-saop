/* Copyright (c) 2017-2018, CNRS-LAAS
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

#include "imc_comm.hpp"

namespace SAOP {
    namespace neptus {

        using boost::asio::ip::tcp;

        void IMCTransportTCP::loop() {
            try {
                for (;;) {
                    boost::asio::io_service io_service;
                    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
                    std::shared_ptr<tcp::socket> sock(new tcp::socket(io_service));
                    std::cout << "Waiting for an incoming connection" << std::endl;
                    tcp::endpoint client_endpoint;
                    a.accept(*sock, client_endpoint);
                    std::cout << "Accepting connection from " << client_endpoint << std::endl;
                    session(std::move(sock));
                }
            } catch (...) {

            }
        }

        /* Run without blocking */
        void IMCTransportTCP::run() {
            session_thread = std::thread(std::bind(&IMCTransportTCP::loop, this));
        }

        void IMCTransportTCP::session(std::shared_ptr<tcp::socket> sock) {
            try {
                std::cout << "Connected" << std::endl;
                IMC::Parser parser = IMC::Parser();
                IMC::ByteBuffer bb;

                for (;;) {
                    // TCP stream reception
                    bb = IMC::ByteBuffer(65535);
                    bb.setSize(65535);
                    boost::system::error_code error;
                    size_t length = sock->read_some(boost::asio::buffer(bb.getBuffer(), bb.getSize()), error);
                    if (error == boost::asio::error::eof)
                        break; // Connection closed cleanly by peer.
                    else if (error)
                        throw boost::system::system_error(error); // Some other error.

                    // IMC message parsing
                    size_t delta = 0;
                    while (delta < length) {
                        auto hb = parser.parse(*(bb.getBuffer() + delta));
                        if (hb != nullptr) {
//                            std::cout << "recv ";
                            size_t ser_size = hb->getSerializationSize();
                            if (recv_handler) {
                                recv_handler(std::move(std::unique_ptr<IMC::Message>(hb)));
                            } else {
                                std::cerr << "recv_handler not set. Received messages are being discarded."
                                          << std::endl;
                            }
                            // Advance buffer cursor past the end of serialized hb.
                            delta += ser_size;
                        } else {
                            delta++;
                        }
                    }

                    std::unique_ptr<IMC::Message> m = nullptr;
                    while (send_q->pop(m)) {
                        IMC::ByteBuffer serl_b = IMC::ByteBuffer(65535);
                        size_t n_bytes = IMC::Packet::serialize(m.get(), serl_b);
                        std::cout << "send " << m->getName() << "(" << static_cast<uint>(m->getId()) << "): "
                                  << "from(" << m->getSource() << ", " << static_cast<uint>(m->getSourceEntity())
                                  << ") " << "to(" << m->getDestination() << ", "
                                  << static_cast<uint>(m->getDestinationEntity()) << ")" << std::endl;
                        boost::asio::write(*sock, boost::asio::buffer(serl_b.getBuffer(), n_bytes));
                    }
                }
            }
            catch (std::exception& e) {
                std::cerr << "Exception in thread: " << e.what() << "\n";
            }
        }

        void IMCCommManager::loop() {
            tcp_server.set_recv_handler(std::bind(&IMCCommManager::message_inbox, this, std::placeholders::_1));

            // Demo HeartBeat handler
            bind<IMC::Heartbeat>([this](std::unique_ptr<IMC::Heartbeat> m) {
                auto answer = std::unique_ptr<IMC::Message>(
                        IMC::Factory::produce(IMC::Factory::getIdFromAbbrev("Heartbeat")));
                answer->setSource(0);
                answer->setSourceEntity(0);
                answer->setDestination(0xFFFF);
                answer->setDestinationEntity(0xFF);
                this->send(std::move(answer));
            });

            tcp_server.run();
            message_dispatching_loop();
        }

        void IMCCommManager::run() {
            message_thread = std::thread(std::bind(&IMCCommManager::loop, this));
        }

        void IMCCommManager::message_dispatching_loop() {
            try {
                for (;;) {
                    std::unique_ptr<IMC::Message> m = nullptr;
                    while (recv_q->wait_pop(m)) {
                        try {
                            auto hnld_fun = message_bindings.at(m->getId());
//                            std::cout << m->getName()
//                                      << "(" << static_cast<uint>(m->getId()) << "): "
//                                      << "from(" << m->getSource() << ", "
//                                      << static_cast<uint>(m->getSourceEntity()) << ") "
//                                      << "to(" << m->getDestination() << ", "
//                                      << static_cast<uint>(m->getDestinationEntity()) << ")"
//                                      << std::endl;
                            auto m_raw = m.get();
                            m.release();
                            // variables can be moved into lambdas only in C++14 onwards, not C++11
                            thp.enqueue([hnld_fun, m_raw] {
                                std::unique_ptr<IMC::Message> mm = std::unique_ptr<IMC::Message>(m_raw);
                                hnld_fun(std::move(mm));
                            });
                        } catch (const std::out_of_range& e) {
                            std::cerr << "UNHANDLED " << m->getName()
                                      << "(" << static_cast<uint>(m->getId()) << "): "
                                      << "from(" << m->getSource() << ", "
                                      << static_cast<uint>(m->getSourceEntity()) << ") "
                                      << "to(" << m->getDestination() << ", "
                                      << static_cast<uint>(m->getDestinationEntity()) << ")"
                                      << std::endl;
                        }
                    }
                }
            } catch (...) {

            }
        }

    }
}