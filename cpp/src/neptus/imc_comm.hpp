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
#include <boost/bind.hpp>
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
        using boost::asio::ip::udp;

        class IMCTransport {
        public:
//            virtual ~IMCTransport() = 0;

            virtual void run() = 0;

            /* Set function the function to be called on message reception */
            virtual void set_recv_handler(std::function<void(std::unique_ptr<IMC::Message>)> a_recv_handler) = 0;

            virtual void send(std::unique_ptr<IMC::Message> message) = 0;

            virtual bool is_ready() = 0;

            virtual void stop() = 0;

        };

        class IMCTransportUDP : public IMCTransport {
            boost::asio::io_service io_service;

            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler;
            std::thread recv_thread;

            udp::endpoint local_endpoint;
            udp::socket recv_socket;
            udp::endpoint recv_endpoint; // End point of the DUNE instance from which we recv messages

            udp::endpoint remote_endpoint;
            udp::socket send_socket;

            std::array<uint8_t, 65535> recv_buffer = std::array<uint8_t, 65535>();

            bool ready = false;

        public:
            explicit IMCTransportUDP(unsigned short port, std::string dst_ip, std::string dst_port)
                    : recv_handler(nullptr),
                      local_endpoint(udp::endpoint(udp::v4(), port)),
                      recv_socket(udp::socket(io_service, local_endpoint)),
                      send_socket(udp::socket(io_service, udp::v4())) {
                udp::resolver resolver(io_service);
                udp::resolver::query query(udp::v4(), dst_ip, dst_port);
                remote_endpoint = *resolver.resolve(query);
            }

            ~IMCTransportUDP() {
                io_service.stop();
                recv_thread.join();
            }

            void run() override {
                recv_thread = std::thread(std::bind(&IMCTransportUDP::recv_io_loop, this));
            }

            /* Set the function to be called on message reception */
            void set_recv_handler(std::function<void(std::unique_ptr<IMC::Message>)> a_recv_handler) override {
                recv_handler = std::move(a_recv_handler);
            }

            /* Enqueue message to be sent */
            void send(std::unique_ptr<IMC::Message> message) override {
                IMC::ByteBuffer serl_b = IMC::ByteBuffer(65535);
                size_t n_bytes = IMC::Packet::serialize(message.get(), serl_b);
                BOOST_LOG_TRIVIAL(debug) << "Send " << message->getName() << "(" << static_cast<uint>(message->getId())
                                         << "): "
                                         << "from (" << message->getSource() << ", "
                                         << static_cast<uint>(message->getSourceEntity())
                                         << ") " << "to (" << message->getDestination() << ", "
                                         << static_cast<uint>(message->getDestinationEntity()) << ")";
                send_socket.send_to(boost::asio::buffer(serl_b.getBuffer(), n_bytes), remote_endpoint);
            }

            bool is_ready() override {
                return ready;
            }

            void stop() override {
                io_service.stop();
                recv_thread.join();
            }

        private:
            void recv_io_loop();

            void async_recv_from();

            void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
        };

        class IMCTransportTCP : public IMCTransport {
            unsigned short port;

            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler;
            std::shared_ptr<IMCMessageQueue> send_q;

            std::thread session_thread;

            bool socket_connected = false;

        public:
            explicit IMCTransportTCP(unsigned short port)
                    : port(port), recv_handler(nullptr), send_q(std::make_shared<IMCMessageQueue>()) {}

            IMCTransportTCP(unsigned short port,
                            std::function<void(std::unique_ptr<IMC::Message>)> recv_handler)
                    : port(port), recv_handler(std::move(recv_handler)),
                      send_q(std::make_shared<IMCMessageQueue>()) {}

            void loop();

            void run() override;

            void session(std::shared_ptr<tcp::socket> sock);

            /* Set function the function to be called on message reception */
            void set_recv_handler(std::function<void(std::unique_ptr<IMC::Message>)> a_recv_handler) override {
                recv_handler = std::move(a_recv_handler);
            }

            /* Enqueue message to be sent */
            void send(std::unique_ptr<IMC::Message> message) override {
                send_q->push(std::move(message));
            }

            bool is_ready() override {
                return socket_connected;
            }

            void stop() override {
                // FIXME
            }
        };

        /* TODO: Enable shared_from_this ? */
        class IMCComm {
            std::unique_ptr<IMCTransport> imc_transport;
            std::shared_ptr<IMCMessageQueue> recv_q;
            std::unordered_map<size_t, std::function<void(std::unique_ptr<IMC::Message>)>> message_bindings;

            std::unique_ptr<std::mutex> binding_mtx = std::unique_ptr<std::mutex>(new std::mutex());

            ThreadPool thp;

        public:
            IMCComm() :
                    imc_transport(std::unique_ptr<IMCTransport>(new IMCTransportTCP(8888))),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    thp(1) {}

            explicit IMCComm(unsigned short port) :
                    imc_transport(std::unique_ptr<IMCTransport>(new IMCTransportTCP(port))),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    thp(1) {}

            IMCComm(unsigned short port, std::string dst_ip, std::string dst_port) :
                    imc_transport(std::unique_ptr<IMCTransport>(new IMCTransportUDP(port, dst_ip, dst_port))),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    thp(1) {}

            explicit IMCComm(std::unique_ptr<IMCTransport> transport) :
                    imc_transport(std::move(transport)),
                    recv_q(std::make_shared<SAOP::neptus::IMCMessageQueue>()),
                    thp(1) {}

//            static std::unique_ptr<IMCComm> using_tcp_transport(unsigned short port) {
//                auto trans = std::unique_ptr<IMCTransport>(new IMCTransportTCP(port));
//                return std::unique_ptr<IMCComm>(new IMCComm(std::move(trans)));
//            }
//
//            static std::unique_ptr<IMCComm> udp_transport(unsigned short port, std::string dst_ip, std::string dst_port) {
//                auto trans = std::unique_ptr<IMCTransport>(new IMCTransportUDP(port, dst_ip, dst_port));
//                return std::unique_ptr<IMCComm>(new IMCComm(std::move(trans)));
//            }

            void loop();

            void run();

            bool is_ready() {
                return ready & imc_transport->is_ready();
            }

            /* Enqueue message to be sent */
            void send(std::unique_ptr<IMC::Message> message) {
                imc_transport->send(std::move(message));
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
            bool ready = false;

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
