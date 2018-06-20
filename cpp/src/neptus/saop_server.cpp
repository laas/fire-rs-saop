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

        GCSCommandOutcome GCS::stop(std::string plan_id, uint16_t uav_addr) {
            auto pc_stop = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_stop->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_stop->op = IMC::PlanControl::OperationEnum::PC_STOP;
            pc_stop->request_id = req_id;
            pc_stop->plan_id = plan_id;

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_stop->toString();

            this->imc_comm->send(std::move(pc_stop));

            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(10),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            auto answer = req.get_answer(req_id);

            if (!answer) {
                return GCSCommandOutcome::Unknown;
            }

            if (*answer == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return GCSCommandOutcome::Success;
            } else {
                return GCSCommandOutcome::Failure;
            }
        }

        GCSCommandOutcome GCS::load(IMC::PlanSpecification ps, uint16_t uav_addr) {
            auto pc_load = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_load->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_load->op = IMC::PlanControl::OperationEnum::PC_LOAD;
            pc_load->request_id = req_id;
            pc_load->plan_id = ps.plan_id;
            pc_load->arg = IMC::InlineMessage<IMC::Message>();
            pc_load->arg.set(ps);

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_load->toString();

            imc_comm->send(std::move(pc_load));

            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(10),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            auto answer = req.get_answer(req_id);

            if (!answer) {
                return GCSCommandOutcome::Unknown;
            }

            if (*answer == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return GCSCommandOutcome::Success;
            } else {
                return GCSCommandOutcome::Failure;
            }
        }

        GCSCommandOutcome GCS::start(std::string plan_id, uint16_t uav_addr) {
//            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);
            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_start->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_start->op = IMC::PlanControl::OperationEnum::PC_START;
            pc_start->request_id = req_id;
            pc_start->plan_id = plan_id;

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_start->toString();

            imc_comm->send(std::move(pc_start));

            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(10),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            auto answer = req.get_answer(req_id);

            if (!answer) {
                return GCSCommandOutcome::Unknown;
            }

            if (*answer == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return GCSCommandOutcome::Success;
            } else {
                return GCSCommandOutcome::Failure;
            }
        }

        GCSCommandOutcome GCS::start(IMC::PlanSpecification ps, uint16_t uav_addr) {
            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_start->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_start->op = IMC::PlanControl::OperationEnum::PC_START;
            pc_start->request_id = req_id;
            pc_start->plan_id = ps.plan_id;
            pc_start->arg = IMC::InlineMessage<IMC::Message>();
            pc_start->arg.set(ps);

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_start->toString();

            imc_comm->send(std::move(pc_start));

            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(10),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            auto answer = req.get_answer(req_id);

            if (!answer) {
                return GCSCommandOutcome::Unknown;
            }

            if (*answer == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                return GCSCommandOutcome::Success;
            } else {
                return GCSCommandOutcome::Failure;
            }
        }

        IMC::PlanSpecification
        GCS::plan_specification(const Plan& saop_plan, size_t trajectory, std::string plan_id) {

            const auto& t = saop_plan.trajectories()[trajectory];

            auto wp = t.as_waypoints();

            // Remove bases
            auto r_start = wp.begin();
            size_t n_start = 0;
            auto r_end = wp.end();
            size_t n_end = t.size();
            if (t.conf().start_position && wp.front() == *t.conf().start_position) {
                r_start++;
                n_start++;
            }
            if (t.conf().end_position && wp.back() == *t.conf().end_position) {
                r_end--;
                n_end--;
            }
            auto wp_filtered = std::vector<Waypoint3d>(r_start, r_end);
            auto wp_wgs84 = lambert93_to_wgs84(wp_filtered);

            std::vector<std::string> maneuver_names = {};

            for (size_t man_id = n_start; man_id < n_end; ++man_id) {
                maneuver_names.emplace_back(std::to_string(man_id));
            }

            return PlanSpecificationFactory::make_message(plan_id, wp_wgs84, maneuver_names);
        }

        void GCS::estimated_state_handler(std::unique_ptr<IMC::EstimatedState> m) {
            UAVStateReport report{m->getTimeStamp(), uav_name_of[m->getSource()],
                                  m->lat, m->lon, m->height,
                                  m->phi, m->theta, m->psi,
                                  m->vx, m->vy, m->vz};
            uav_report_handler(report);
        }

        void GCS::plan_control_state_handler(std::unique_ptr<IMC::PlanControlState> m) {
            TrajectoryExecutionState state = TrajectoryExecutionState::Blocked;
            switch (static_cast<IMC::PlanControlState::StateEnum>(m->state)) {
                case IMC::PlanControlState::StateEnum::PCS_BLOCKED:
                    state = TrajectoryExecutionState::Blocked;
                    break;
                case IMC::PlanControlState::StateEnum::PCS_EXECUTING:
                    state = TrajectoryExecutionState::Executing;
                    break;
                case IMC::PlanControlState::StateEnum::PCS_INITIALIZING:
                    state = TrajectoryExecutionState::Executing;
                    break;
                case IMC::PlanControlState::StateEnum::PCS_READY:
                    state = TrajectoryExecutionState::Ready;
                    break;
                default:
                    break;
            }

            TrajectoryExecutionOutcome outcome = TrajectoryExecutionOutcome::Nothing;
            switch (static_cast<IMC::PlanControlState::LastPlanOutcomeEnum>(m->last_outcome)) {
                case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_NONE:
                    outcome = TrajectoryExecutionOutcome::Nothing;
                    break;
                case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_SUCCESS :
                    outcome = TrajectoryExecutionOutcome::Success;
                    break;
                case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_FAILURE :
                    outcome = TrajectoryExecutionOutcome::Failure;
                    break;
                default:
                    break;
            }

            // See pt.lsts.neptus.console.plugins.planning.PlanControlStatePanel for "state" and "last outcome" interpretation

            TrajectoryExecutionReport report{m->getTimeStamp(), m->plan_id, uav_name_of[m->getSource()], m->man_id,
                                             m->man_eta, state, outcome};
            m->last_outcome;
            plan_report_handler(report);
        }

        void GCS::plan_control_handler(std::unique_ptr<IMC::PlanControl> m) {
            BOOST_LOG_TRIVIAL(debug) << "Handling PlanControl answer to request " << m->request_id <<
                                     " (op " << static_cast<IMC::PlanControl::OperationEnum>(m->op) <<
                                     " type " << static_cast<IMC::PlanControl::TypeEnum>(m->type) <<
                                     ": " << m->info << ")";
            auto t = static_cast<IMC::PlanControl::TypeEnum>(m->type);
            if ((t == IMC::PlanControl::TypeEnum::PC_SUCCESS) || (t == IMC::PlanControl::TypeEnum::PC_FAILURE)) {
                // Notify Plan control success or failure
                std::unique_lock<std::mutex> lock(pc_answer_mutex);
                if (req.set_answer(m->request_id, t)) {
                    BOOST_LOG_TRIVIAL(info) << "PlanControl request " << m->request_id <<
                                            " op " << static_cast<IMC::PlanControl::OperationEnum>(m->op) <<
                                            " replied " << static_cast<IMC::PlanControl::TypeEnum>(m->type) <<
                                            " : " << m->info << ")";
                    pc_answer_cv.notify_all();
                } else {
                    BOOST_LOG_TRIVIAL(warning) << "Received reply for unexpected PlanControl request: "
                                               << m->request_id
                                               << " (op " << static_cast<IMC::PlanControl::OperationEnum>(m->op) <<
                                               " type " << static_cast<IMC::PlanControl::TypeEnum>(m->type) << ")";
                }
            } else {
                // IMC::PlanControl::TypeEnum::PC_IN_PROGRESS
                // Keep waiting for the final answer
                BOOST_LOG_TRIVIAL(debug) << "Keep waiting for the final outcome of request: " << m->request_id;
            }
        }

        GCSCommandOutcome GCS::load(const Plan& p, size_t trajectory, std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return GCSCommandOutcome::Failure;
            }

            return load(plan_specification(p, trajectory, plan_id), uav_addr);
        }

        GCSCommandOutcome GCS::start(const Plan& p, size_t trajectory, std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return GCSCommandOutcome::Failure;
            }

            return start(plan_specification(p, trajectory, plan_id), uav_addr);
        }

        GCSCommandOutcome GCS::start(std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return GCSCommandOutcome::Failure;
            }
            return start(plan_id, uav_addr);

        }

        GCSCommandOutcome GCS::stop(std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return GCSCommandOutcome::Failure;
            }
            return stop(plan_id, uav_addr);
        }
    }
}
#endif //PLANNING_CPP_SAOP_NEPTUS_H