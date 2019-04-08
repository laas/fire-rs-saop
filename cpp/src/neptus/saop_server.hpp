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
#ifndef PLANNING_CPP_SAOP_SERVER_HPP
#define PLANNING_CPP_SAOP_SERVER_HPP

#include <array>
#include <condition_variable>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <numeric>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "../../IMC/Base/Message.hpp"
#include "../../IMC/Spec/EstimatedState.hpp"
#include "../../IMC/Spec/Heartbeat.hpp"
#include "../../IMC/Spec/PlanControl.hpp"
#include "../../IMC/Spec/PlanControlState.hpp"
#include "../../IMC/Spec/DevDataBinary.hpp"

#include "../ext/WGS84.hpp"

#include "../vns/plan.hpp"

#include "imc_comm.hpp"
#include "imc_message_factories.hpp"
#include "geography.hpp"

namespace IMC {

    [[maybe_unused]]
    static std::ostream& operator<<(std::ostream& stream, const IMC::PlanControlState::StateEnum& pcs) {
        switch (pcs) {
            case IMC::PlanControlState::StateEnum::PCS_READY:
                stream << "READY";
                break;
            case IMC::PlanControlState::StateEnum::PCS_INITIALIZING:
                stream << "INITIALIZING";
                break;
            case IMC::PlanControlState::StateEnum::PCS_EXECUTING:
                stream << "EXECUTING";
                break;
            case IMC::PlanControlState::StateEnum::PCS_BLOCKED:
                stream << "BLOCKED";
                break;
        }
        return stream;
    }

    [[maybe_unused]]
    static std::ostream& operator<<(std::ostream& stream, const IMC::PlanControlState::LastPlanOutcomeEnum& lpo) {
        switch (lpo) {
            case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_NONE:
                stream << "NONE";
                break;
            case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_SUCCESS:
                stream << "SUCCESS";
                break;
            case IMC::PlanControlState::LastPlanOutcomeEnum::LPO_FAILURE:
                stream << "FAILURE";
                break;
        }
        return stream;
    }

    [[maybe_unused]]
    static std::ostream& operator<<(std::ostream& stream, const IMC::PlanControl::TypeEnum& pc) {
        switch (pc) {
            case IMC::PlanControl::TypeEnum::PC_REQUEST:
                stream << "REQUEST";
                break;
            case IMC::PlanControl::TypeEnum::PC_SUCCESS:
                stream << "SUCCESS";
                break;
            case IMC::PlanControl::TypeEnum::PC_FAILURE:
                stream << "FAILURE";
                break;
            case IMC::PlanControl::TypeEnum::PC_IN_PROGRESS:
                stream << "IN PROGRESS";
                break;
        }
        return stream;
    }

    [[maybe_unused]]
    static std::ostream& operator<<(std::ostream& stream, const IMC::PlanControl::OperationEnum& lpo) {
        switch (lpo) {
            case IMC::PlanControl::OperationEnum::PC_START:
                stream << "START";
                break;
            case IMC::PlanControl::OperationEnum::PC_STOP:
                stream << "STOP";
                break;
            case IMC::PlanControl::OperationEnum::PC_LOAD:
                stream << "LOAD";
                break;
            case IMC::PlanControl::OperationEnum::PC_GET:
                stream << "GET";
                break;
        }
        return stream;
    }
}


namespace SAOP {

    namespace neptus {

        template<typename Payload>
        class Requests {
            uint16_t last_req_id = 1000;
            std::unordered_map<uint16_t, std::queue<Payload>> answers = std::unordered_map<uint16_t, std::queue<Payload>>();
            std::unique_ptr<std::mutex> req_id_mutex = std::unique_ptr<std::mutex>(new std::mutex());

        public:
            Requests() = default;

            Requests(const Requests& r) = delete;

            Requests& operator=(const Requests& other) = delete;

            /* Obtain a request id and keep it until the answer is retrieved */
            uint16_t request_id() {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);

                //FIXME: This will loop inf if there are max uint16_t requests unanswered (unlikely)
                while (answers.find(last_req_id) != answers.end()) {
                    last_req_id += 1;
                }

                answers[last_req_id] = std::queue<Payload>();
                return last_req_id++;
            }

            /* Set a request as answered.
             * Returns false if the req_id is not tracked as unanswered */
            bool set_answer(uint16_t req_id, Payload p) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = answers.find(req_id);
                if (req_id_it == answers.end()) {
                    return false;
                } else {
                    answers[req_id].push(p);
                    return true;
                }
            }

            /* Retrieve the answer of a request if it exists */
            opt<Payload> get_answer(uint16_t req_id) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = answers.find(req_id);
                if (req_id_it != answers.end()) {
                    if (!answers[req_id].empty()) {
                        auto p = answers[req_id].front();
                        answers[req_id].pop();
                        return p;
                    }
                }
                return opt<Payload>();
            }

            /* Get wether the answer exists */
            bool has_answer(uint16_t req_id) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = answers.find(req_id);
                if (req_id_it != answers.end()) {
                    if (answers[req_id].empty()) {
                        return false;
                    }
                }
                return true;
            }

            void dismiss(uint16_t req_id) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the answers set
                auto req_id_it = answers.find(req_id);
                if (req_id_it != answers.end()) {
                    answers.erase(req_id_it);
                }
            }
        };


        enum class TrajectoryExecutionState : uint8_t {
            Blocked, // Plan execution is blocked due to a uav failure
            Ready, // Plan not running
            Executing, // Plan is running
        };

        static std::ostream& operator<<(std::ostream& stream, const TrajectoryExecutionState& tes) {
            switch (tes) {
                case TrajectoryExecutionState::Blocked:
                    stream << "Blocked";
                    break;
                case TrajectoryExecutionState::Ready:
                    stream << "Ready";
                    break;
                case TrajectoryExecutionState::Executing:
                    stream << "Executing";
                    break;
            }
            return stream;
        }

        enum class TrajectoryExecutionOutcome : uint8_t {
            Nothing, // No plan has been completed yet
            Success, // If excution state is Ready or Blocked, the last plan was completed successfully
            Failure, // If excution state is Ready or Blocked, the last plan failed
        };

        static std::ostream& operator<<(std::ostream& stream, const TrajectoryExecutionOutcome& teo) {
            switch (teo) {
                case TrajectoryExecutionOutcome::Nothing:
                    stream << "None";
                    break;
                case TrajectoryExecutionOutcome::Success:
                    stream << "Success";
                    break;
                case TrajectoryExecutionOutcome::Failure:
                    stream << "Failure";
                    break;
            }
            return stream;
        }


        // Neptus plans correspond to SAOP Plan trajectories
        // A SAOP Plan can be seen as set of Neptus plans
        struct TrajectoryExecutionReport {
            double timestamp;
            std::string plan_id; // Neptus plan name
            std::string uav; // Neptus uav performing the trajectory
            std::string maneuver; // Current maneuver being executed
            int32_t maneuver_eta; // Estimated time until completion of the manuever
            TrajectoryExecutionState state; // Execution status
            TrajectoryExecutionOutcome last_outcome; // Execution status

            friend std::ostream& operator<<(std::ostream& stream, const TrajectoryExecutionReport& ter) {
                stream << "TrajectoryExecutionReport("
                       << ter.timestamp << ", "
                       << ter.plan_id << ", "
                       << ter.uav << ", "
                       << ter.maneuver << ", "
                       << ter.maneuver_eta << ", "
                       << ter.state << ", "
                       << ter.last_outcome << ")";
                // TODO Add uav list
                return stream;
            }
        };

        struct UAVStateReport {
            // Summary of IMC::EstimatedState
            double timestamp;
            std::string uav; // UAV id
            double lat; // WGS84 latitude (rad)
            double lon; // WGS84 longitude (rad)
            double height; // WGS84 altitude asl (m)
            double phi; // Roll (rad)
            double theta; // Pitch (rad)
            double psi; // Yaw (rad)
            double vx; // North (x) ground speed (m/s)
            double vy; // East (y) ground speed (m/s)
            double vz; // Down (z) ground speed (m/s)

            friend std::ostream& operator<<(std::ostream& stream, const UAVStateReport& usr) {
                stream << "UAVStateReport("
                       << usr.timestamp << ", "
                       << usr.uav << ", "
                       << usr.lat << ", "
                       << usr.lon << ", "
                       << usr.height << ", "
                       << usr.phi << ", "
                       << usr.theta << ", "
                       << usr.psi << ", "
                       << usr.vx << ", "
                       << usr.vy << ", "
                       << usr.vz << ")";
                return stream;
            }
        };

        // Neptus plans correspond to SAOP Plan trajectories
        // A SAOP Plan can be seen as set of Neptus plans

        struct FireMapReport {
            double timestamp;
            std::string uav;
            DRaster firemap;

            friend std::ostream& operator<<(std::ostream& stream, const FireMapReport& fmr) {
                stream << "FireMapReport("
                       << fmr.timestamp << ", "
                       << fmr.uav << ", "
                       << fmr.firemap << ")";
                return stream;
            }
        };

        /* Command and supervise plan execution */
        class GCS final {
        public:
            explicit GCS(std::shared_ptr<IMCComm> imc) :
                    GCS(std::move(imc), nullptr, nullptr, nullptr) {}

            GCS(std::shared_ptr<IMCComm> imc,
                std::function<void(TrajectoryExecutionReport per)> plan_report_cb,
                std::function<void(UAVStateReport usr)> uav_report_cb,
                std::function<void(FireMapReport fmr)> firemap_report_cb,
                int pcs_epsg = EPSG_RGF93_LAMBERT93)
                    : imc_comm(std::move(imc)),
                      plan_report_handler(std::move(plan_report_cb)),
                      uav_report_handler(std::move(uav_report_cb)),
                      firemap_report_handler(std::move(firemap_report_cb)),
                      projected_coordinate_system_epsg(pcs_epsg),
                      req() {

                BOOST_LOG_TRIVIAL(info) << "Using projected coordinate system " << projected_coordinate_system_epsg;
                // Bind to IMC messages
                imc_comm->bind<IMC::EstimatedState>(
                        std::bind(&GCS::estimated_state_handler, this, placeholders::_1));
                imc_comm->bind<IMC::PlanControlState>(
                        std::bind(&GCS::plan_control_state_handler, this, placeholders::_1));
                imc_comm->bind<IMC::PlanControl>(
                        std::bind(&GCS::plan_control_handler, this, placeholders::_1));
                imc_comm->bind<IMC::DevDataBinary>(
                        std::bind(&GCS::dev_data_binary_handler, this, placeholders::_1));
            }

            virtual ~GCS() {
                imc_comm->unbind<IMC::DevDataBinary>();
                imc_comm->unbind<IMC::PlanControl>();
                imc_comm->unbind<IMC::PlanControlState>();
                imc_comm->unbind<IMC::EstimatedState>();
            }

            GCS(const GCS&) = delete;

            GCS(GCS&&) = delete;

            GCS& operator=(const GCS&) = delete;

            GCS& operator=(GCS&&) = delete;

            /* Send a request to Neptus to load a converted version of SAOP::Plan. */
            bool load(const Plan& p, size_t trajectory, std::string plan_id, std::string uav);

            /* Load and start a SAOP::Plan. */
            bool start(const Plan& p, size_t trajectory, std::string plan_id, std::string uav);

            /* Load and start a SAOP::Trajectory. */
            bool start(const Trajectory& t, std::string uav);

            /* Start the last loaded PlanSpecification */
            bool start(std::string plan_id, std::string uav);

            /* Perform a loiter maneuver */
            bool loiter(std::string plan_id, LoiterManeuver loiter, double speed, std::string uav);

            /* Stop the plan currently being executed. */
            bool stop(std::string plan_id, std::string uav);

            bool set_wind(double modulo, double direction, std::string uav);

            /* Return true if the connection with the GCS is available */
            bool is_ready() {
                return imc_comm->is_ready();
            }

            std::vector<std::string> available_vehicles() {
                auto v = std::vector<std::string>();
                for (const auto& t : available_uavs) {
                    v.emplace_back(std::get<1>(t));
                }
                return v;
            }

        private:
            std::shared_ptr<IMCComm> imc_comm;
            std::thread exec_thread;

            mutable std::mutex pc_answer_mutex;
            mutable std::condition_variable pc_answer_cv;

            /* Function to be called periodically during execution, carrying plan execution reports */
            std::function<void(TrajectoryExecutionReport per)>
                    plan_report_handler;
            /* Function to be called periodically with UAV state information*/
            std::function<void(UAVStateReport usr)> uav_report_handler;
            /* Function to be called with received firemaps*/
            std::function<void(FireMapReport usr)> firemap_report_handler;

            int projected_coordinate_system_epsg = EPSG_RGF93_LAMBERT93;

            std::unordered_map<uint16_t, std::string> uav_name_of = {{0x0c0c, "x8-02"},
                                                                     {0x0c10, "x8-06"}};

            std::unordered_map<std::string, uint16_t> uav_addr_of = {{"x8-02", 0x0c0c},
                                                                     {"x8-06", 0x0c10}};

            std::vector<std::tuple<uint16_t, std::string>> available_uavs = {std::make_tuple(0x0c0c, "x8-02"),
                                                                             std::make_tuple(0x0c10, "x8-06")};
            Requests<IMC::PlanControl> req;

            /* Send the PlanControl load request for a PlanSpecification */
            bool load(IMC::PlanSpecification ps, uint16_t uav_addr);

            /* Load and start a PlanSpecification*/
            bool start(IMC::PlanSpecification ps, uint16_t uav_addr);

            bool start(std::string plan_id, uint16_t uav_addr);

            /* Send a stop command for plan_id to uav_addr*/
            bool stop(std::string plan_id, uint16_t uav_addr);

            bool wait_for_load(uint16_t req_id);

            bool wait_for_stop(uint16_t req_id);

            bool wait_for_start(uint16_t req_id);

            IMC::PlanSpecification
            plan_specification(const Plan& saop_plan, size_t trajectory, std::string plan_id);

            IMC::PlanSpecification plan_specification(const Trajectory& t);

            IMC::PlanSpecification plan_specification(const Trajectory& t, std::string plan_id);

            IMC::PlanSpecification plan_specification(LoiterManeuver loiter, double speed, std::string plan_id);

            void estimated_state_handler(std::unique_ptr<IMC::EstimatedState> m);

            void plan_control_state_handler(std::unique_ptr<IMC::PlanControlState> m);

            void plan_control_handler(std::unique_ptr<IMC::PlanControl> m);

            /* Handle fire map messages from the fire mapper */
            void dev_data_binary_handler(std::unique_ptr<IMC::DevDataBinary> m);
        };

    }
}

#endif //PLANNING_CPP_SAOP_SERVER_HPP
