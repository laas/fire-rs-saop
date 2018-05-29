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

#include "../vns/plan.hpp"

#include "imc_comm.hpp"
#include "imc_message_factories.hpp"
#include "geography.hpp"

namespace SAOP {

    namespace neptus {

        template<typename Payload>
        class Requests {
            uint16_t last_req_id = 0;
            std::unordered_map<uint16_t, opt<Payload>> unanswered = std::unordered_map<uint16_t, opt<Payload>>();
            std::unique_ptr<std::mutex> req_id_mutex = std::unique_ptr<std::mutex>(new std::mutex());

        public:
            Requests() = default;

            Requests(const Requests& r) = delete;

            Requests& operator=(const Requests& other) = delete;

            /* Obtain a request id and keep it until the answer is retrieved */
            uint16_t request_id() {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);

                //FIXME: This will loop inf if there are max uint16_t requests unanswered (unlikely)
                while (unanswered.find(last_req_id) != unanswered.end()) {
                    last_req_id += 1;
                }

                unanswered[last_req_id] = opt<Payload>();
                return last_req_id++;
            }

            /* Set a request as answered.
             * Returns false if the req_id is not tracked as unanswered */
            bool set_answer(uint16_t req_id, Payload p) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = unanswered.find(req_id);
                if (req_id_it == unanswered.end()) {
                    return false;
                } else {
                    unanswered[req_id] = p;
                    return true;
                }
            }

            /* Retrieve the answer of a request if it exists */
            opt<Payload> get_answer(uint16_t req_id) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = unanswered.find(req_id);
                if (req_id_it != unanswered.end()) {
                    if (unanswered[req_id].has_value()) {
                        auto p = unanswered[req_id];
                        unanswered.erase(req_id_it);
                        return p;
                    }
                }
                return opt<Payload>();
            }

            /* Get wether the answer exists */
            bool has_answer(uint16_t req_id) {
                std::lock_guard<std::mutex> lock_m(*req_id_mutex);
                // req_id must exist in the unanswered set
                auto req_id_it = unanswered.find(req_id);
                if (req_id_it != unanswered.end()) {
                    if (unanswered[req_id].has_value()) {
                        return true;
                    }
                }
                return false;
            }
        };

        enum class PlanExecutionState : uint8_t {
            None,
            Ready,
            Executing, // Plan is still running
            Failure, // Plan execution failed
            Success, // Plan successfully executed
        };

        struct PlanExecutionReport {
            // Summary of IMC::PlanControlState
            double timestamp;
            std::string plan_id;
            PlanExecutionState state;
            std::vector<std::string> vehicles;
            // TODO: current maneuver
            // TODO: ETA

            friend std::ostream& operator<<(std::ostream& stream, const PlanExecutionReport& per) {
                stream << "PlanExecutionReport("
                       << per.timestamp << ", "
                       << per.plan_id << ", "
                       << static_cast<uint8_t>(per.state) << ")";
                // TODO Add vehicle list
                return stream;
            }
        };

        struct UAVStateReport {
            // Summary of IMC::EstimatedState
            double timestamp;
            uint16_t uav_id; // UAV id
            double lat; // WGS84 latitude (rad)
            double lon; // WGS84 longitude (rad)
            float height; // WGS84 altitude asl (m)
            float phi; // Roll (rad)
            float theta; // Pitch (rad)
            float psi; // Yaw (rad)
            float vx; // North (x) ground speed (m/s)
            float vy; // East (y) ground speed (m/s)
            float vz; // Down (z) ground speed (m/s)

            friend std::ostream& operator<<(std::ostream& stream, const UAVStateReport& usr) {
                stream << "UAVStateReport("
                       << usr.timestamp << ", "
                       << usr.uav_id << ", "
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

        enum class GCSCommandOutcome {
            Unknown = 2,
            Success = 1,
            Failure = 0,
        };

        /* Command and supervise plan execution */
        class GCS final {
        public:
            explicit GCS(std::shared_ptr<IMCComm> imc) :
                    GCS(std::move(imc), nullptr, nullptr) {}

            GCS(std::shared_ptr<IMCComm> imc,
                std::function<void(PlanExecutionReport per)> plan_report_cb,
                std::function<void(UAVStateReport usr)> uav_report_cb)
                    : imc_comm(std::move(imc)),
                      plan_report_handler(std::move(plan_report_cb)),
                      uav_report_handler(std::move(uav_report_cb)),
                      req() {

                // FIXME Test calls
                plan_report_handler(PlanExecutionReport());
                uav_report_handler(UAVStateReport());

                // Bind to IMC messages
                imc_comm->bind<IMC::EstimatedState>(
                        std::bind(&GCS::estimated_state_handler, this, placeholders::_1));
                imc_comm->bind<IMC::PlanControlState>(
                        std::bind(&GCS::plan_control_state_handler, this, placeholders::_1));
                imc_comm->bind<IMC::PlanControl>(
                        std::bind(&GCS::plan_control_handler, this, placeholders::_1));
            }

            ~GCS() {
                imc_comm->unbind<IMC::PlanControl>();
                imc_comm->unbind<IMC::PlanControlState>();
                imc_comm->unbind<IMC::EstimatedState>();
            }

            GCS(const GCS&) = delete;

            GCS(GCS&&) = delete;

            GCS& operator=(const GCS&) = delete;

            GCS& operator=(GCS&&) = delete;

//            /* Setup, and run inmediately, a SAOP plan.
//             * (Not blocking) */
//            void execute(const Plan& plan);

            /* Send a request to Neptus to load a converted version of SAOP::Plan. */
            GCSCommandOutcome load(const Plan& p);

            /* Load and start a SAOP::Plan. */
            GCSCommandOutcome start(const Plan& p);

            /* Start the last loaded PlanSpecification */
            GCSCommandOutcome start();

            /* Stop the plan currently being executed. */
            GCSCommandOutcome stop();

            /* Return true if this GCS is not active. */
            bool is_active() {
                //FIXME use timestamp from last message to determine if it is running or not
                return true;
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
            std::function<void(PlanExecutionReport per)>
                    plan_report_handler;
            /* Function to be called periodically with UAV state information*/
            std::function<void(UAVStateReport usr)> uav_report_handler;

            std::vector<std::tuple<uint16_t, std::string>> available_uavs = {{0x0c0c, "x8-02"},
                                                                             {0x0c10, "x8-06"}};

            std::string plan_id = "plan"; // TODO: make customizable?
            Requests<IMC::PlanControl::TypeEnum> req;

            /* Send the PlanControl load request for a PlanSpecification */
            GCSCommandOutcome load(IMC::PlanSpecification ps);

            /* Load and start a PlanSpecification*/
            GCSCommandOutcome start(IMC::PlanSpecification ps);

            IMC::PlanSpecification plan_specification(const Plan& saop_plan, size_t trajectory);

//            void plan_execution_loop(IMC::PlanSpecification imc_plan);

            void estimated_state_handler(std::unique_ptr<IMC::EstimatedState> m);

            void plan_control_state_handler(std::unique_ptr<IMC::PlanControlState> m);

            void plan_control_handler(std::unique_ptr<IMC::PlanControl> m);
        };

    }
}

#endif //PLANNING_CPP_SAOP_SERVER_HPP
