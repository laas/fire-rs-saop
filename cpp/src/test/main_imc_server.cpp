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
#include <iostream>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../../IMC/Base/Message.hpp"
#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Spec/Enumerations.hpp"
#include "../../IMC/Spec/PlanManeuver.hpp"
#include "../../IMC/Spec/PlanSpecification.hpp"
#include "../../IMC/Spec/PlanTransition.hpp"
#include "../../IMC/Spec/Goto.hpp"
#include "../../IMC/Spec/EntityParameter.hpp"
#include "../../IMC/Spec/EntityParameters.hpp"
#include "../../IMC/Spec/SetEntityParameters.hpp"
#include "../../IMC/Spec/Announce.hpp"

using boost::asio::ip::tcp;
using boost::asio::ip::udp;

const int max_length = 1024;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

IMC::Announce create_announce() {
    auto announce = IMC::Announce();
    announce.sys_name = "ccu-test-158-226";
    announce.sys_type = IMC::SystemType::SYSTEMTYPE_CCU;
    announce.owner = 65535;
    announce.services = "imc+tcp://193.137.158.226:60000";

    return announce;
}

void send_announce(const std::string &host, const std::string &port) {
    boost::asio::io_service io_service;

    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));

    udp::resolver resolver(io_service);
    udp::resolver::query query(udp::v4(), host, port);
    udp::resolver::iterator iterator = resolver.resolve(query);
    auto bb = IMC::ByteBuffer();
    auto pk = IMC::Packet();
    auto ann = create_announce();
    pk.serialize(&ann, bb);
    s.send_to(boost::asio::buffer(bb.getBuffer(), bb.getSize()), *iterator);
}

IMC::PlanSpecification create_imc_plan() {

    auto h_ctrl_set_entity_parameters = IMC::SetEntityParameters();
    h_ctrl_set_entity_parameters.name = "Height Control";
    h_ctrl_set_entity_parameters.params = IMC::MessageList<IMC::EntityParameter>();
    auto h_ctrl_entity_parameter = IMC::EntityParameter();
    h_ctrl_entity_parameter.name = "Active";
    h_ctrl_entity_parameter.value = "true";
    h_ctrl_set_entity_parameters.params.push_back(h_ctrl_entity_parameter);

    auto p_ctrl_set_entity_parameters = IMC::SetEntityParameters();
    p_ctrl_set_entity_parameters.name = "Path Control";
    p_ctrl_set_entity_parameters.params = IMC::MessageList<IMC::EntityParameter>();
    auto p_ctrl_entity_parameter = IMC::EntityParameter();
    p_ctrl_entity_parameter.name = "Use controller";
    p_ctrl_entity_parameter.value = "true";
    p_ctrl_set_entity_parameters.params.push_back(h_ctrl_entity_parameter);

    auto start_actions = IMC::MessageList<IMC::Message>();
    start_actions.push_back(h_ctrl_set_entity_parameters);
    start_actions.push_back(p_ctrl_set_entity_parameters);

    auto man_goto0 = IMC::Goto();
    man_goto0.timeout=10000;
    man_goto0.lat = 0.73050675;
    man_goto0.lon = -0.11706505;
    man_goto0.z = 800;
    man_goto0.z_units = IMC::ZUnits::Z_HEIGHT;
    man_goto0.speed = 17.0;
    man_goto0.speed_units = IMC::SpeedUnits::SUNITS_METERS_PS;
    man_goto0.roll = -1;
    man_goto0.pitch = -1;
    man_goto0.yaw = -1;
    man_goto0.custom = "tuplelist";


    auto pm = IMC::PlanManeuver();
    pm.maneuver_id = "Goto0";
    pm.data = IMC::InlineMessage<IMC::Maneuver>();
    pm.data.set(man_goto0);
    pm.start_actions = start_actions;

    auto pt = IMC::PlanTransition();
    pt.source_man = "Goto0";
    pt.dest_man = "Goto0";
    pt.conditions = "ManeuverIsDone";

    auto plan_spec = IMC::PlanSpecification();
    plan_spec.plan_id = "Simple plan";
    plan_spec.start_man_id = "Goto0";
    plan_spec.maneuvers = IMC::MessageList<IMC::PlanManeuver>();
    plan_spec.maneuvers.push_back(pm);
    plan_spec.transitions = IMC::MessageList<IMC::PlanTransition>();
    plan_spec.transitions.push_back(pt);

    return plan_spec;
}


void session(socket_ptr sock)
{
    try
    {
        for (;;)
        {
//            char data[max_length];
            IMC::ByteBuffer bb = IMC::ByteBuffer();
            bb.setSize(max_length);

            boost::system::error_code error;
            size_t length = sock->read_some(boost::asio::buffer(bb.getBuffer(), bb.getSize()), error);
            if (error == boost::asio::error::eof)
                break; // Connection closed cleanly by peer.
            else if (error)
                throw boost::system::system_error(error); // Some other error.

            // Deserialize IMC packet
            auto pk = IMC::Packet();
            auto hb = pk.deserialize(bb.getBuffer(), bb.getSize());
            // Can I cast a Message to its message type? Heartbeat for instance

            std::cout << "Received: " << hb->toString() << std::endl;

//            // Invert source and destination
//            auto src = hb->getSource();
//            auto src_ent = hb->getSourceEntity();
//            auto dst = hb->getDestination();
//            auto dst_ent = hb->getDestinationEntity();
//            hb->setDestination(src);
//            hb->setDestinationEntity(src_ent);
//            hb->setSource(dst);
//            hb->setSourceEntity(dst_ent);

            // Serialize the message
            auto hbb = create_imc_plan();
            pk.serialize(&hbb, bb);

            std::cout << "Sent: " << hbb.toString() << std::endl;

            // Send
            boost::asio::write(*sock, boost::asio::buffer(bb.getBuffer(), bb.getSize()));
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception in thread: " << e.what() << "\n";
    }
}

void server(boost::asio::io_service& io_service, short port)
{
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
    for (;;)
    {
        socket_ptr sock(new tcp::socket(io_service));
        a.accept(*sock);
        boost::thread t(boost::bind(session, sock));
    }
}

int main(int argc, char* argv[])
{
    try
    {
        if (argc != 2)
        {
            std::cerr << "Usage: blocking_tcp_echo_server <port>\n";
            return 1;
        }

        // Send announce to dune
//        send_announce("127.0.0.1", "6002");
//        send_announce("193.137.158.226", "6002");
//        send_announce("193.137.159.255", "6002");

        boost::asio::io_service io_service;

        using namespace std; // For atoi.
        server(io_service, atoi(argv[1]));
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}