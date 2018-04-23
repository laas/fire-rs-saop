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

//    template<typename T>
//    class SharedQueue<T> {
//        std::queue<T> q;
//        std::mutex m;
//    };


    namespace neptus {
        using boost::asio::ip::tcp;

        class IMCServerTCP {
        private:
            unsigned short port;

            std::function<void(IMC::Message&)> recv_handler;
            std::shared_ptr<std::queue<std::unique_ptr<IMC::Message>>> send_q;
        public:

            explicit IMCServerTCP(unsigned short port, std::function<void(IMC::Message&)> recv_handler,
                                  std::shared_ptr<std::queue<std::unique_ptr<IMC::Message>>> send_queue)
                    : port(port), recv_handler(std::move(recv_handler)), send_q(std::move(send_queue)) {}

            void run() {
                for (;;) {
                    boost::asio::io_service io_service;
                    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
                    std::shared_ptr<tcp::socket> sock(new tcp::socket(io_service));
                    std::cout << "waiting to accept connection" << std::endl;
                    a.accept(*sock);
                    boost::thread t(boost::bind(&IMCServerTCP::session, this, std::move(sock)));
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
                        bb.setSize(1024);
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
                                std::cout << hb->getName() << std::endl;
                                recv_handler(*hb);
                            }
                            delta++;
                        }

                        while (!send_q->empty()) {
                            std::unique_ptr<IMC::Message> m = std::move(send_q->front());
                            send_q->pop();

                            m->setSource(24290);
                            m->setSourceEntity(8);
                            m->setDestination(3088);
                            m->setDestinationEntity(2);
                            m->setTimeStamp();

                            IMC::ByteBuffer serl_b = IMC::ByteBuffer(65535);
                            size_t n_bytes = IMC::Packet::serialize(m.get(), serl_b);
                                std::cout << "Sending: " << m->getName() << std::endl;
                            boost::asio::write(*sock, boost::asio::buffer(serl_b.getBuffer(), n_bytes));
                        }
                    }
                }
                catch (std::exception& e) {
                    std::cerr << "Exception in thread: " << e.what() << "\n";
                }
            }
        };
    }
}


void user_input_loop(std::shared_ptr<std::queue<std::unique_ptr<IMC::Message>>> send_q) {
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
            send_q->emplace(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
        } else if (u_input == "sp") {
            auto a_plan = SAOP::neptus::PlanControlFactory::make_start_plan_message(plan_spec_message.plan_id);
            send_q->emplace(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
        } else {
            std::cout << "Wrong command" << std::endl;
        }
    }
}


int main() {
    std::function<void(IMC::Message&)> recv_handler = [](IMC::Message& m) {
        std::cout << "Received: " << m.getSource() << " " << static_cast<uint>(m.getSourceEntity()) << " "
                  << m.getDestination() << " " << static_cast<uint>(m.getDestinationEntity()) << std::endl;
    };

    auto send_q = std::make_shared<std::queue<std::unique_ptr<IMC::Message>>>();

    SAOP::neptus::IMCServerTCP s(8888, recv_handler, send_q);

    auto plan_spec_message = SAOP::neptus::PlanSpecificationFactory::make_message();
    auto a_plan = SAOP::neptus::PlanControlFactory::make_load_plan_message(plan_spec_message, 12345);
    send_q->emplace(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
//    s.run();

    boost::thread t(boost::bind(&SAOP::neptus::IMCServerTCP::run, s));
    user_input_loop(send_q);
    t.join();
}


#endif //PLANNING_CPP_SAOP_NEPTUS_H
