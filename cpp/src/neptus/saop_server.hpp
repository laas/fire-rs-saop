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

        enum class PlanExecutionState {
            Executing, // Plan is still running
            Success, // Plan successfully executed
            Failure, // Plan execution failed
        };

        class PlanExecutionReport {
            // Summary of IMC::PlanControlState
            std::chrono::system_clock::time_point timestamp;
            std::string plan_id;
            PlanExecutionState state;
            std::vector<std::string> vehicles;
            // TODO: current maneuver
            // TODO: ETA
        };

        class UAVStateReport {
            // Summary of IMC::EstimatedState
            uint16_t id; // UAV id
            double lat; // WGS84 latitude (rad)
            double lon; // WGS84 longitude (rad)
            double height; // WGS84 altitude asl (m)
            double phi; // Roll (rad)
            double theta; // Pitch (rad)
            double psi; // Yaw (rad)
            double vx; // North (x) ground speed (m/s)
            double vy; // East (y) ground speed (m/s)
            double vz; // Down (z) ground speed (m/s)
        };

        /* Command and supervise plan execution */
        class PlanExecutionManager {
        public:
            explicit PlanExecutionManager(std::shared_ptr<IMCCommManager> imc) :
                    PlanExecutionManager(std::move(imc), nullptr, nullptr) {}

            PlanExecutionManager(std::shared_ptr<IMCCommManager> imc,
                                 std::function<void(PlanExecutionReport per)> plan_report_cb,
                                 std::function<void(UAVStateReport usr)> uav_report_cb)
                    : imc_comm(std::move(imc)),
                      plan_report_handler(std::move(plan_report_cb)),
                      uav_report_handler(std::move(uav_report_cb)) {}

            /* Setup, and run inmediately, a SAOP plan
             * (Not blocking) */
            void execute(const Plan& plan);

            /* Stop the plan currently being executed. */
            void stop();

            /* Return true if this PlanExecutionManager is not active. */
            bool is_active() {
                return exec_thread.joinable();
            }

        private:
            std::shared_ptr<IMCCommManager> imc_comm;
            std::thread exec_thread;

            /* Function to be called periodically during execution, carrying plan execution reports */
            std::function<void(PlanExecutionReport per)> plan_report_handler;
            /* Function to be called periodically with UAV state information*/
            std::function<void(UAVStateReport usr)> uav_report_handler;

            std::vector<std::tuple<uint16_t, std::string>> available_uavs = {{0x0c0c, "x8-02"},
                                                                             {0x0c10, "x8-06"}};

            std::string plan_id = "saop_plan"; // TODO: make customizable?
            const static uint16_t req_id_stop = 0x570D;

            IMC::PlanSpecification plan_specification(const Plan& saop_plan, size_t trajectory);

            void plan_execution_loop(IMC::PlanSpecification imc_plan);
        };

    }
}

#endif //PLANNING_CPP_SAOP_SERVER_HPP
