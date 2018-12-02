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

        bool GCS::stop(std::string plan_id, uint16_t uav_addr) {
            auto pc_stop = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_stop->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_stop->op = IMC::PlanControl::OperationEnum::PC_STOP;
            pc_stop->request_id = req_id;
            pc_stop->plan_id = plan_id;

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_stop->toString();

            this->imc_comm->send(std::move(pc_stop));

            return wait_for_stop(req_id);
        }

        bool GCS::load(IMC::PlanSpecification ps, uint16_t uav_addr) {
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

            return wait_for_load(req_id);
        }

        bool GCS::start(std::string plan_id, uint16_t uav_addr) {
//            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, 0x0c10, 0xFF);
            auto pc_start = produce_unique<IMC::PlanControl>(0, 0, uav_addr, 0xFF);

            auto req_id = req.request_id();

            pc_start->type = IMC::PlanControl::TypeEnum::PC_REQUEST;
            pc_start->op = IMC::PlanControl::OperationEnum::PC_START;
            pc_start->request_id = req_id;
            pc_start->plan_id = plan_id;

            BOOST_LOG_TRIVIAL(debug) << "Send " << pc_start->toString();

            imc_comm->send(std::move(pc_start));

            return wait_for_start(req_id);
        }

        bool GCS::start(IMC::PlanSpecification ps, uint16_t uav_addr) {
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

            return wait_for_load(req_id) && wait_for_start(req_id);
        }

        IMC::PlanSpecification
        GCS::plan_specification(const Plan& saop_plan, size_t trajectory, std::string plan_id,
                                int epsg_pcs = EPSG_RGF93_LAMBERT93) {

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

            std::vector<Waypoint3d> wp_wgs84;
            switch (epsg_pcs) {
                case EPSG_ETRS89_LAEA:
                    wp_wgs84 = laea_to_world_coordinates(wp_filtered);
                    break;
                default:
                    // If not ETRS89/LAEA, fallback to RGF93/Lambert93
                    wp_wgs84 = lambert93_to_world_coordinates(wp_filtered);
            }

            std::vector<std::string> maneuver_names = {};

            for (size_t man_id = n_start; man_id < n_end; ++man_id) {
                maneuver_names.emplace_back(std::to_string(man_id));
            }

            return PlanSpecificationFactory::make_message(plan_id, wp_wgs84, maneuver_names);
        }

        IMC::PlanSpecification
        GCS::plan_specification(const Trajectory& t, int epsg_pcs = EPSG_RGF93_LAMBERT93) {
            // epsg_pcs: EPSG code of trajectory waypoints projected coordinate system
            auto wp = t.as_waypoints();
            auto all_names = t.names();

            // Remove bases
            auto r_start = wp.begin();
            auto n_start = all_names.begin();
            auto r_end = wp.end();
            auto n_end = all_names.end();
            if (t.conf().start_position && wp.front() == *t.conf().start_position) {
                r_start++;
                n_start++;
            }
            if (t.conf().end_position && wp.back() == *t.conf().end_position) {
                r_end--;
                n_end--;
            }
            auto wp_filtered = std::vector<Waypoint3d>(r_start, r_end);

            std::vector<Waypoint3d> wp_wgs84;
            switch (epsg_pcs) {
                case EPSG_ETRS89_LAEA:
                    wp_wgs84 = laea_to_world_coordinates(wp_filtered);
                    break;
                default:
                    // If not ETRS89/LAEA, fallback to RGF93/Lambert93
                    wp_wgs84 = lambert93_to_world_coordinates(wp_filtered);
            }

            std::vector<std::string> maneuver_names = std::vector<std::string>(n_start, n_end);


            return PlanSpecificationFactory::make_message(t.name(), wp_wgs84, maneuver_names);
        }

        void GCS::estimated_state_handler(std::unique_ptr<IMC::EstimatedState> m) {

            // Convert from DUNE frame to WGS84
            // Base location "LLH"
            double m_lat = m->lat;
            double m_lon = m->lon;
            double m_agl = m->height;
            // m->x, m->y, m->z is a displacement from LLH
            DUNE::Coordinates::WGS84::displace(m->x, m->y, m->z, &m_lat, &m_lon, &m_agl);

            UAVStateReport report{m->getTimeStamp(), uav_name_of[m->getSource()],
                                  m_lat, m_lon, m->alt,
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
                if (req.set_answer(m->request_id, IMC::PlanControl(*m))) {
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

        bool GCS::load(const Plan& p, size_t trajectory, std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return false;
            }

            return load(plan_specification(p, trajectory, plan_id), uav_addr);
        }

        bool GCS::start(const Plan& p, size_t trajectory, std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return false;
            }

            return start(plan_specification(p, trajectory, plan_id), uav_addr);
        }

        bool GCS::start(const Trajectory& t, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return false;
            }

            return start(plan_specification(t), uav_addr);
        }

        bool GCS::start(std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return false;
            }
            return start(plan_id, uav_addr);

        }

        bool GCS::stop(std::string plan_id, std::string uav) {
            uint16_t uav_addr = 0;
            auto uav_id_it = uav_addr_of.find(uav);
            if (uav_id_it != uav_addr_of.end()) {
                uav_addr = uav_id_it->second;
            } else {
                BOOST_LOG_TRIVIAL(error) << "UAV \"" << uav << "\" is unknown";
                return false;
            }
            return stop(plan_id, uav_addr);
        }

        bool GCS::wait_for_load(uint16_t req_id) {
            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(30),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            while (req.has_answer(req_id)) {
                auto answer = req.get_answer(req_id);

                if (!answer) {
                    return false;
                }

                BOOST_LOG_TRIVIAL(debug) << (*answer).plan_id << " req " << (*answer).request_id << " "
                                         << static_cast<IMC::PlanControl::OperationEnum>((*answer).op) << " "
                                         << static_cast<IMC::PlanControl::TypeEnum>((*answer).type) << " "
                                         << (*answer).info;

                if ((*answer).info == "plan loaded") {
                    if ((*answer).type == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                        return true;
                    } else { //(*answer).type == IMC::PlanControl::TypeEnum::PC_FAILURE

                        return false;
                    }
                }
            }
            return false;
        }

        bool GCS::wait_for_stop(uint16_t req_id) {
            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(30),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            while (req.has_answer(req_id)) {
                auto answer = req.get_answer(req_id);

                if (!answer) {
                    return false;
                }

                BOOST_LOG_TRIVIAL(debug) << (*answer).plan_id << " req " << (*answer).request_id << " "
                                         << static_cast<IMC::PlanControl::OperationEnum>((*answer).op) << " "
                                         << static_cast<IMC::PlanControl::TypeEnum>((*answer).type) << " "
                                         << (*answer).info;

                if ((*answer).type == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                    return true;
                } else {
                    //(*answer).type == IMC::PlanControl::TypeEnum::PC_FAILURE
                    // Neptus considers stopping nothing as a failure. For us this is a success.
                    if ((*answer).info == "no plan is running, request ignored") {
                        return true;
                    }
                    return false;
                }
            }
            return false;
        }

        bool GCS::wait_for_start(uint16_t req_id) {
            // Wait for an answer from Neptus. If none is received, assume a failure
            std::unique_lock<std::mutex> lock(pc_answer_mutex);
            pc_answer_cv.wait_for(lock, std::chrono::seconds(30),
                                  [this, req_id]() -> bool { return this->req.has_answer(req_id); });

            while (req.has_answer(req_id)) {
                auto answer = req.get_answer(req_id);

                if (!answer) {
                    return false;
                }

                BOOST_LOG_TRIVIAL(debug) << (*answer).plan_id << " req " << (*answer).request_id << " "
                                         << static_cast<IMC::PlanControl::OperationEnum>((*answer).op) << " "
                                         << static_cast<IMC::PlanControl::TypeEnum>((*answer).type) << " "
                                         << (*answer).info;

                if ((*answer).info.find("executing maneuver") != std::string::npos) {
                    if ((*answer).type == IMC::PlanControl::TypeEnum::PC_SUCCESS) {
                        return true;
                    } else { //(*answer).type == IMC::PlanControl::TypeEnum::PC_FAILURE
                        return false;
                    }
                }
            }
            return false;
        }
    }
}
#endif //PLANNING_CPP_SAOP_NEPTUS_H