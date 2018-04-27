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
#include "saop_server.hpp"

void user_input_loop(std::shared_ptr<SAOP::neptus::IMCCommManager> imc_comm) {
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
            imc_comm->send(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
        } else if (u_input == "sp") {
            auto a_plan = SAOP::neptus::PlanControlFactory::make_start_plan_message(plan_spec_message.plan_id);
            imc_comm->send(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));
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

    std::function<void(std::unique_ptr<IMC::EstimatedState>)> recv_estimatedstate = [](
            std::unique_ptr<IMC::EstimatedState> m) {
        std::cout << "Received: " << m->getSource() << " " << static_cast<uint>(m->getSourceEntity()) << " "
                  << m->getDestination() << " " << static_cast<uint>(m->getDestinationEntity()) << std::endl;
    };

    std::function<void(std::unique_ptr<IMC::PlanControl>)> recv_plancontrol = [](
            std::unique_ptr<IMC::PlanControl> m) {
        m->toJSON(std::cout);
        std::cout << std::endl;
    };

    std::function<void(std::unique_ptr<IMC::PlanControlState>)> recv_plancontrolstate = [](
            std::unique_ptr<IMC::PlanControlState> m) {
        m->toJSON(std::cout);
        std::cout << std::endl;
    };

    std::shared_ptr<SAOP::neptus::IMCCommManager> imc_comm = std::make_shared<SAOP::neptus::IMCCommManager>();

    // Start IMCCommManager in a different thread
    auto t = std::thread(std::bind(&SAOP::neptus::IMCCommManager::loop, std::ref(imc_comm)));

    // Demo EstimatedState handler
    imc_comm->bind<IMC::EstimatedState>(recv_estimatedstate);
    imc_comm->bind<IMC::PlanControl>(recv_plancontrol);
    imc_comm->bind<IMC::PlanControlState>(recv_plancontrolstate);

    // Send a demo plan
    auto plan_spec_message = SAOP::neptus::PlanSpecificationFactory::make_message();
    auto a_plan = SAOP::neptus::PlanControlFactory::make_load_plan_message(plan_spec_message, 12345);
    imc_comm->send(std::unique_ptr<IMC::Message>(new IMC::PlanControl(std::move(a_plan))));

    user_input_loop(imc_comm);


}