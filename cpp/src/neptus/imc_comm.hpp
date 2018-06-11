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
#ifndef PLANNING_CPP_IMC_SERVER_HPP
#define PLANNING_CPP_IMC_SERVER_HPP

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <boost/log/trivial.hpp>

#include "../../IMC/Base/ByteBuffer.hpp"
#include "../../IMC/Base/Message.hpp"
#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Base/Parser.hpp"
#include "../../IMC/Spec/EstimatedState.hpp"
#include "../../IMC/Spec/Heartbeat.hpp"
#include "../../IMC/Spec/PlanControl.hpp"
#include "../../IMC/Spec/PlanControlState.hpp"

#include "../core/SharedQueue.hpp"
#include "../ext/ThreadPool.hpp"
#include "../vns/plan.hpp"

#include "imc_message_factories.hpp"

namespace SAOP {
    namespace neptus {

        typedef SharedQueue<std::unique_ptr<IMC::Message>> IMCMessageQueue;

        using boost::asio::ip::tcp;

        class IMCTransportTCP {
            unsigned short port;

            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler;
            std::shared_ptr<IMCMessageQueue> send_q;

            std::thread session_thread;

        public:
            explicit IMCTransportTCP(unsigned short port)
                    : port(port), recv_handler(nullptr), send_q(std::make_shared<IMCMessageQueue>()) {}

            IMCTransportTCP(unsigned short port,
                            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler)
                    : port(port), recv_handler(std::move(recv_handler)), send_q(std::make_shared<IMCMessageQueue>()) {}

            void loop();

            void run();

            void session(std::shared_ptr<tcp::socket> sock);

            /* Set function the function to be called on message reception */
            void set_recv_handler(std::function<void(std::unique_ptr<IMC::Message>)> a_recv_handler) {
                recv_handler = std::move(a_recv_handler);
            }

            /* Enqueue message to be sent */
            void send(std::unique_ptr<IMC::Message> message) {
                send_q->push(std::move(message));
            }
        };

        /* TODO: Enable shared_from_this ? */
        class IMCComm {
            IMCTransportTCP tcp_server;
            std::shared_ptr<IMCMessageQueue> recv_q;
            std::unordered_map<size_t, std::function<void(std::unique_ptr<IMC::Message>)>> message_bindings;

            std::unique_ptr<std::mutex> binding_mtx = std::unique_ptr<std::mutex>(new std::mutex());

            ThreadPool thp;

        public:
            IMCComm() :
                    tcp_server(IMCTransportTCP(8888)),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    thp(1) {}

//            explicit IMCComm(IMCTransportTCP tcp_server) :
//                    tcp_server(std::move(tcp_server)),
//                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()) {}

            void loop();

            void run();

            /* Enqueue message to be sent */
            void send(std::unique_ptr<IMC::Message> message) {
                tcp_server.send(std::move(message));
            }

            /*Bind a message id to a handler function*/
            template<typename M>
            void bind(std::function<void(std::unique_ptr<M>)> message_handler) {
                // std::function arguments cannot be casted. We define a wrapper instead
                std::function<void(std::unique_ptr<IMC::Message>)> g_func = [message_handler](
                        std::unique_ptr<IMC::Message> m) {
                    message_handler(std::unique_ptr<M>(static_cast<M*>(m.release())));

                };

                std::unique_lock<std::mutex> lock(*binding_mtx);
                if (message_bindings.count(M::getIdStatic()) > 0) {
                    std::cerr << "Replacing existing binding for "
                              << IMC::Factory::getAbbrevFromId(M::getIdStatic())
                              << std::endl;
                }
                bind(M::getIdStatic(), g_func);
            }

            template<typename M>
            void unbind() {
                std::unique_lock<std::mutex> lock(*binding_mtx);
                unbind(M::getIdStatic());
            }

        private:
            std::thread message_thread;

            /* Bind a message id to a handler function */
            void bind(size_t id, std::function<void(std::unique_ptr<IMC::Message>)> message_handler) {
                message_bindings[id] = std::move(message_handler);
            }

            void unbind(size_t id) {
                message_bindings.erase(id);
            }

            void message_dispatching_loop();

            void message_inbox(std::unique_ptr<IMC::Message> m) {
                recv_q->push(std::move(m));
            }

        };
    }
}

#endif //PLANNING_CPP_IMC_SERVER_HPP
