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

#ifndef PLANNING_CPP_IMCMESSAGEFACTORY_H
#define PLANNING_CPP_IMCMESSAGEFACTORY_H

#include "../../IMC/Base/Message.hpp"
#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Spec/Announce.hpp"
#include "../../IMC/Spec/EntityParameter.hpp"
#include "../../IMC/Spec/EntityParameters.hpp"
#include "../../IMC/Spec/Enumerations.hpp"
#include "../../IMC/Spec/Goto.hpp"
#include "../../IMC/Spec/PlanDB.hpp"
#include "../../IMC/Spec/PlanManeuver.hpp"
#include "../../IMC/Spec/PlanSpecification.hpp"
#include "../../IMC/Spec/PlanTransition.hpp"
#include "../../IMC/Spec/SetEntityParameters.hpp"

namespace SAOP {

    namespace Exec {

        class StartActionsFactory{
        public:
            static IMC::MessageList<IMC::Message> make_message() {
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
                p_ctrl_set_entity_parameters.params.push_back(p_ctrl_entity_parameter);

                auto start_actions = IMC::MessageList<IMC::Message>();
                start_actions.push_back(h_ctrl_set_entity_parameters);
                start_actions.push_back(p_ctrl_set_entity_parameters);

                return start_actions;
            };
        };

        class PlanManeuverFactory {
        public:
            static IMC::PlanManeuver make_message(const std::string &maneuver_id, const IMC::Goto &goto_message) {
                auto pm = IMC::PlanManeuver();
                pm.maneuver_id = maneuver_id;
                pm.data = IMC::InlineMessage<IMC::Maneuver>();
                pm.data.set(goto_message);
                pm.start_actions = StartActionsFactory::make_message();

                return pm;
            }
        };

        class GotoFactory {
        public:
            static IMC::Goto make_message(uint16_t timeout, double lat, double lon,
                                          float z, uint8_t z_units,
                                          float speed, uint8_t speed_units,
                                          double roll, double pitch, double yaw,
                                          const std::string &custom) {
                auto maneuver_goto = IMC::Goto();
                maneuver_goto.timeout = timeout;
                maneuver_goto.lat = lat;
                maneuver_goto.lon = lon;
                maneuver_goto.z = z;
                maneuver_goto.z_units = z_units;
                maneuver_goto.speed = speed;
                maneuver_goto.speed_units = speed_units;
                maneuver_goto.roll = roll;
                maneuver_goto.pitch = pitch;
                maneuver_goto.yaw = yaw;
                maneuver_goto.custom = custom;
                return maneuver_goto;
            };

            static IMC::Goto make_message(double lat, double lon, float z) {
                return make_message(1000,lat, lon, z, IMC::ZUnits::Z_HEIGHT,
                                    17.0, IMC::SpeedUnits::SUNITS_METERS_PS,
                                    0, 0, 0, "tuplelist");
            };
        };

        class SequentialPlanTransitionListFactory {
        public:
            static IMC::MessageList<IMC::PlanTransition> make_message(const std::vector<IMC::PlanManeuver> &maneuvers){
                auto transitions = IMC::MessageList<IMC::PlanTransition>();

                for(size_t i = 0; i < maneuvers.size()-1; ++i) {
                    auto pt = IMC::PlanTransition();
                    pt.source_man = maneuvers[i].maneuver_id;
                    pt.dest_man = maneuvers[i+1].maneuver_id;
                    pt.conditions = "ManeuverIsDone";
                    transitions.push_back(pt);
                }
                return transitions;

            }
        };

        class PlanSpecificationFactory {
        public:
            static IMC::PlanSpecification make_message() {

                auto man_goto0 = SAOP::Exec::GotoFactory::make_message(0.73050675, -0.11706505, 800);
                auto man_goto1 = SAOP::Exec::GotoFactory::make_message(0.73060675, -0.11706505, 800);
                auto man_goto2 = SAOP::Exec::GotoFactory::make_message(0.73060675, -0.11716505, 800);

                auto pm0 = SAOP::Exec::PlanManeuverFactory::make_message("Goto0", man_goto0);
                auto pm1 = SAOP::Exec::PlanManeuverFactory::make_message("Goto1", man_goto1);
                auto pm2 = SAOP::Exec::PlanManeuverFactory::make_message("Goto2", man_goto2);

                auto plan_spec = IMC::PlanSpecification();
                plan_spec.plan_id = "Simple plan";
                plan_spec.start_man_id = "Goto0";
                plan_spec.maneuvers = IMC::MessageList<IMC::PlanManeuver>();
                plan_spec.maneuvers.push_back(pm0);
                plan_spec.maneuvers.push_back(pm1);
                plan_spec.maneuvers.push_back(pm2);
                plan_spec.transitions = SequentialPlanTransitionListFactory::make_message({pm0, pm1, pm2});

                return plan_spec;
            }

        };

        class PlanDBFactory {
        public:
            static IMC::PlanDB make_message() {
                auto plan_spec = SAOP::Exec::PlanSpecificationFactory::make_message();
                auto pdb = IMC::PlanDB();
                pdb.type = IMC::PlanDB::TypeEnum::DBT_REQUEST;
                pdb.op = IMC::PlanDB::OperationEnum::DBOP_SET;
                pdb.request_id = 0;
                pdb.plan_id = plan_spec.plan_id;
                pdb.arg = IMC::InlineMessage<IMC::Message>();
                pdb.arg.set(plan_spec);
                pdb.info = "some useless info";

                return pdb;
            }
        };
    }
}

#endif //PLANNING_CPP_IMCMESSAGEFACTORY_H
