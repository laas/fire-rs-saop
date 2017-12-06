#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#include "../../IMC/Spec/Heartbeat.hpp"
#include "../../IMC/Base/Packet.hpp"

using boost::asio::ip::tcp;

enum { max_length = 1024 };
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


int main(int argc, char* argv[])
{
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: blocking_tcp_echo_client <host> <port>\n";
            return 1;
        }

        boost::asio::io_service io_service;

        // Connect to the server
        tcp::socket s(io_service);
        tcp::resolver resolver(io_service);
        boost::asio::connect(s, resolver.resolve({argv[1], argv[2]}));

        std::cout << "Enter message: ";
        char request[max_length];
        std::cin.getline(request, max_length);
//        size_t request_length = std::strlen(request);

//        auto hb = std::unique_ptr<IMC::Heartbeat>(new IMC::Heartbeat());
        auto hb = create_imc_plan();
        hb.setTimeStamp();
        auto pk = IMC::Packet();
        IMC::ByteBuffer bb = IMC::ByteBuffer();
        pk.serialize(static_cast<IMC::Message*>(&hb), bb);


        // Send message
        boost::asio::write(s, boost::asio::buffer(bb.getBuffer(), bb.getSize()));

//        // Get reply
//        char reply[max_length];
//        size_t reply_length = boost::asio::read(s,
//                                                boost::asio::buffer(reply, bb.getSize()));
//        std::cout << "Reply is: ";
//        std::cout.write(reply, reply_length);
//        std::cout << "\n";
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}

