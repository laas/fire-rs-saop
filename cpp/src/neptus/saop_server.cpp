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

#include <array>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "../../IMC/Base/ByteBuffer.hpp"
#include "../../IMC/Base/Message.hpp"
#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Base/Parser.hpp"

#include "imc_message_factories.hpp"


namespace SAOP {

    template<typename T>
    class SharedQueue {
        std::queue<T> q;
        std::mutex m;

    public:
        SharedQueue<T>() = default;

        /* Retrieve the first element
         * returns true if the queue wasn't empty and false otherwise.*/
        bool pop(T& x) {
            std::lock_guard<std::mutex> lock(m);
            if (!q.empty()) {
                x = std::move(q.front());
                q.pop();
                return true;
            }
            return false;
        }

        /*Put an element at the end of the queue*/
        void push(T&& x) {
            std::lock_guard<std::mutex> lock(m);
            q.push(std::move(x));
        }

    };

    namespace neptus {

        typedef SharedQueue<std::unique_ptr<IMC::Message>> IMCMessageQueue;

        using boost::asio::ip::tcp;

        class IMCTransportTCP {
        private:
            unsigned short port;

            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler;
            std::shared_ptr<IMCMessageQueue> send_q;

        public:
            explicit IMCTransportTCP(unsigned short port)
                    : port(port), recv_handler(nullptr), send_q(std::make_shared<IMCMessageQueue>()) {}

            explicit IMCTransportTCP(unsigned short port,
                                     std::function<void(std::unique_ptr<IMC::Message>)> recv_handler)
                    : port(port), recv_handler(std::move(recv_handler)), send_q(std::make_shared<IMCMessageQueue>()) {}


            void run() {
                try {
                    for (;;) {
                        boost::asio::io_service io_service;
                        tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
                        std::shared_ptr<tcp::socket> sock(new tcp::socket(io_service));
                        std::cout << "Waiting for an incoming connection" << std::endl;
                        tcp::endpoint client_endpoint;
                        a.accept(*sock, client_endpoint);
                        std::cout << "Accepting connection from " << client_endpoint << std::endl;
                        //boost::thread t(boost::bind(&IMCServerTCP::session, this, std::move(sock)));
                        session(std::move(sock));
                    }
                } catch (...) {

                }
            }

            void session(std::shared_ptr<tcp::socket> sock) {
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
                                std::cout << "recv ";
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

            void set_recv_handler(std::function<void(std::unique_ptr<IMC::Message>)> a_recv_handler) {
                recv_handler = std::move(a_recv_handler);
            }

            /*Enqueue message to be sent*/
            void send(std::unique_ptr<IMC::Message> message) {
                send_q->push(std::move(message));
            }
        };

        class IMCCommManager {
            IMCTransportTCP tcp_server;

            std::shared_ptr<IMCMessageQueue> recv_q;
            std::shared_ptr<IMCMessageQueue> send_q;

        public:

            explicit IMCCommManager() :
                    tcp_server(IMCTransportTCP(8888)),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    send_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()) {}

            explicit IMCCommManager(IMCTransportTCP tcp_server) :
                    tcp_server(std::move(tcp_server)),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    send_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()) {}

            void run() {
                tcp_server.set_recv_handler(std::bind(&IMCCommManager::handle_message, this, std::placeholders::_1));
                auto t = std::thread(std::bind(&IMCTransportTCP::run, tcp_server));
                for (;;) {};
            }

        private:
            void handle_message(std::unique_ptr<IMC::Message> m) {
                std::cout << m->getName() << "(" << static_cast<uint>(m->getId()) << "): "
                          << "from(" << m->getSource() << ", " << static_cast<uint>(m->getSourceEntity()) << ") "
                          << "to(" << m->getDestination() << ", " << static_cast<uint>(m->getDestinationEntity()) << ")"
                          << std::endl;
                if (m->getId() == IMC::Factory::getIdFromAbbrev("Heartbeat")) {
                    // Reply with another heartbeat
                    auto m_hb = std::unique_ptr<IMC::Message>(IMC::Factory::produce("Heartbeat"));
                    m_hb->setSource(0);
                    m_hb->setSourceEntity(0);
                    m_hb->setDestination(0xFFFF);
                    m_hb->setDestinationEntity(0xFF);
                    tcp_server.send(std::move(m));
                }
            }

        };
    }
}


void user_input_loop(std::shared_ptr<SAOP::neptus::IMCMessageQueue> send_q) {
    bool exit = false;
    auto plan_spec_message = SAOP::neptus::PlanSpecificationFactory::make_message();

    while (!exit) {
        std::cout << "Send message: ";
        std::string u_input;
        std::cin >> u_input;
        std::cout << std::endl;
        if (u_input == "e") {
            exit = !exit;
        } else if (u_input == "lp") {
            auto a_plan = SAOP::neptus::PlanControlFactory::make_load_plan_message(plan_spec_message, 12345);
            send_q->push(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
        } else if (u_input == "sp") {
            auto a_plan = SAOP::neptus::PlanControlFactory::make_start_plan_message(plan_spec_message.plan_id);
            send_q->push(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
        } else {
            std::cout << "Wrong command" << std::endl;
        }
    }
}


int main() {
    std::function<void(std::unique_ptr<IMC::Message>)> recv_handler = [](std::unique_ptr<IMC::Message> m) {
        std::cout << "Received: " << m->getSource() << " " << static_cast<uint>(m->getSourceEntity()) << " "
                  << m->getDestination() << " " << static_cast<uint>(m->getDestinationEntity()) << std::endl;
    };


//    SAOP::neptus::IMCTransportTCP s(8888, recv_handler);

//    auto plan_spec_message = SAOP::neptus::PlanSpecificationFactory::make_message();
//    auto a_plan = SAOP::neptus::PlanControlFactory::make_load_plan_message(plan_spec_message, 12345);
////    send_q->push(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
////    s.run();

//    boost::thread t(boost::bind(&SAOP::neptus::IMCTransportTCP::run, s));

    SAOP::neptus::IMCCommManager imc_comm = SAOP::neptus::IMCCommManager();

    auto t = std::thread(std::bind(&SAOP::neptus::IMCCommManager::run, imc_comm));

//    imc_comm.run();

//    user_input_loop(send_q);
    t.join();
}


#endif //PLANNING_CPP_SAOP_NEPTUS_H
