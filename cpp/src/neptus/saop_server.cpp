/* Copyright (c) 2018, CNRS-LAAS
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

#include "saop_server.hpp"


namespace SAOP {

    namespace neptus {


//        void PlanExecutionManager::execute(const Plan& plan) {
//
////            exec_thread = std::thread(std::bind(&PlanExecutionManager::plan_execution_loop,
////                                                this, ps_message));
//        }

        bool PlanExecutionManager::stop() {
            auto pc_stop = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);

            pc_stop->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_stop->op = IMC::PlanControl::OperationEnum::PC_STOP;
            pc_stop->request_id = req_id_stop;
            pc_stop->plan_id = plan_id;

            this->imc_comm->send(std::move(pc_stop));

            // Wait for an answer from Neptus
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait(lock, [this] { return req_answer; });
            auto r = *req_answer;
            req_answer = {};

            if (r == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return true;
            } else {
                return false;
            }
        }

        bool PlanExecutionManager::load(IMC::PlanSpecification ps) {
            auto pc_load = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);

            pc_load->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_load->op = IMC::PlanControl::OperationEnum::PC_LOAD;
            pc_load->request_id = req_id_load;
            pc_load->plan_id = plan_id;
            pc_load->arg = IMC::InlineMessage<IMC::Message>();
            pc_load->arg.set(ps);

            imc_comm->send(std::move(pc_load));

            // Wait for an answer from Neptus
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait(lock, [this] { return req_answer; });
            auto r = *req_answer;
            req_answer = {};

            if (r == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return true;
            } else {
                return false;
            }
        }

        bool PlanExecutionManager::start() {
//            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);
            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, 0x0c0c, 0xFF);

            pc_start->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_start->op = IMC::PlanControl::OperationEnum::PC_START;
            pc_start->request_id = req_id_start;
            pc_start->plan_id = "saop_plan";

            req_answer = {}; // Reset PlanControl answer before sending an start msg.

            imc_comm->send(std::move(pc_start));

            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(5), [this]() -> bool { return req_answer.has_value(); });

            if (!req_answer) {
                return false;
            } else {
                req_answer = {};
            }

            if (*req_answer == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return true;
            } else {
                return false;
            }
        }


        bool PlanExecutionManager::start(IMC::PlanSpecification ps) {
            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);

            pc_start->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_start->op = IMC::PlanControl::OperationEnum::PC_START;
            pc_start->request_id = req_id_start;
            pc_start->plan_id = plan_id;
            pc_start->arg = IMC::InlineMessage<IMC::Message>();
            pc_start->arg.set(ps);

            imc_comm->send(std::move(pc_start));

            // Wait for an answer from Neptus
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait(lock, [this] { return req_answer; });
            auto r = *req_answer;
            req_answer = {};

            if (r == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return true;
            } else {
                return false;
            }
        }

        IMC::PlanSpecification
        PlanExecutionManager::plan_specification(const Plan& saop_plan, size_t trajectory) {

            const auto& t = saop_plan.trajectories()[trajectory];

            auto wp = t.as_waypoints();

            // Remove bases
            auto r_start = wp.begin();
            auto r_end = wp.end();
            if (t.conf().start_position && wp.front() == *t.conf().start_position) {
                r_start++;
            }
            if (t.conf().end_position && wp.back() == *t.conf().end_position) {
                r_end--;
            }
            auto wp_filtered = std::vector<Waypoint3d>(r_start, r_end);

            auto wp_wgs84 = WGS84_waypoints(wp_filtered, Position(690487, 4636304));

            return PlanSpecificationFactory::make_message(plan_id, wp_wgs84);
        }

        void PlanExecutionManager::estimated_state_handler(std::unique_ptr<IMC::EstimatedState> m) {
            UAVStateReport report{m->getTimeStamp(), m->getSource(),
                                  m->lat, m->lon, m->height,
                                  m->phi, m->theta, m->psi,
                                  m->vx, m->vy, m->vz};
            uav_report_handler(report);
        }

        void PlanExecutionManager::plan_control_state_handler(std::unique_ptr<IMC::PlanControlState> m) {
            PlanExecutionReport report = PlanExecutionReport();
            plan_report_handler(report);
        }

        void PlanExecutionManager::plan_control_handler(std::unique_ptr<IMC::PlanControl> m) {
            std::cout << "req: " << m->request_id << static_cast<uint16_t>(m->type) << std::endl;
            auto t = static_cast<IMC::PlanControl::TypeEnum>(m->type);
            if (t == IMC::PlanControl::TypeEnum::PC_SUCCESS || t == IMC::PlanControl::TypeEnum::PC_FAILURE) {
                // Notify Plan control success or failure
                std::unique_lock<std::mutex> lock(pc_answer_mutex);
                req_answer = t;
                pc_answer_cv.notify_one();
            } else {
                //Keep waiting for the final answer
                std::cout << "Keep waiting for the final answer" << std::endl;
            }

        }

        bool PlanExecutionManager::load(const Plan& p) {
            if (p.trajectories().size() > 1) {
                // TODO FIXME
                std::cerr << "Plans for fleets of UAV are not supported yet";
            }
            return load(plan_specification(p, 0));
        }

        bool PlanExecutionManager::start(const Plan& p) {
            if (p.trajectories().size() > 1) {
                // TODO FIXME
                std::cerr << "Plans for fleets of UAV are not supported yet";
            }
            return start(plan_specification(p, 0));
        }
    }
}
#endif //PLANNING_CPP_SAOP_NEPTUS_H