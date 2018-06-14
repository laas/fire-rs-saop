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
#include "../../IMC/Spec/Loiter.hpp"
#include "../../IMC/Spec/PlanControl.hpp"
#include "../../IMC/Spec/PlanDB.hpp"
#include "../../IMC/Spec/PlanManeuver.hpp"
#include "../../IMC/Spec/PlanSpecification.hpp"
#include "../../IMC/Spec/PlanTransition.hpp"
#include "../../IMC/Spec/SetEntityParameters.hpp"
#include "../../IMC/Spec/WindSpeed.hpp"

#include "../core/waypoint.hpp"

namespace SAOP {

    namespace neptus {

        template<typename M>
        std::unique_ptr<M> produce_unique(uint16_t src, uint8_t src_ent, uint16_t dst, uint8_t dst_ent) {
            auto m = std::unique_ptr<M>(new M());
            m->setTimeStamp();
            m->setSource(src);
            m->setSourceEntity(src_ent);
            m->setDestination(dst);
            m->setDestinationEntity(dst_ent);
            return m;
        }

        class StartActionsFactory {
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
            static IMC::PlanManeuver make_message(const std::string& maneuver_id,
                                                  const IMC::Maneuver& maneuver_message) {
                auto pm = IMC::PlanManeuver();
                pm.maneuver_id = maneuver_id;
                pm.data = IMC::InlineMessage<IMC::Maneuver>();
                pm.data.set(maneuver_message);
                pm.start_actions = StartActionsFactory::make_message();

                return pm;
            }
        };

        class GotoFactory {
        public:
            static IMC::Goto make_message(uint16_t timeout, double lat, double lon, float z, uint8_t z_units,
                                          float speed, uint8_t speed_units, double roll, double pitch, double yaw,
                                          const std::string& custom) {
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
                return make_message(1000, lat, lon, z, IMC::ZUnits::Z_HEIGHT,
                                    17.0, IMC::SpeedUnits::SUNITS_METERS_PS,
                                    0, 0, 0, "tuplelist");
            };
        };

        class LoiterFactory {
        public:
            static IMC::Loiter make_message(uint16_t timeout, double lat, double lon, float z, uint8_t z_units,
                                            uint16_t duration, float speed, uint8_t speed_units, uint8_t type,
                                            float radius, float length, double bearing, uint8_t direction,
                                            const std::string& custom) {
                auto maneuver_loiter = IMC::Loiter();
                maneuver_loiter.timeout = timeout;
                maneuver_loiter.lat = lat;
                maneuver_loiter.lon = lon;
                maneuver_loiter.z = z;
                maneuver_loiter.z_units = z_units;
                maneuver_loiter.duration = duration;
                maneuver_loiter.speed = speed;
                maneuver_loiter.speed_units = speed_units;
                maneuver_loiter.type = type;
                maneuver_loiter.radius = radius;
                maneuver_loiter.length = length;
                maneuver_loiter.bearing = bearing;
                maneuver_loiter.direction = direction;
                maneuver_loiter.custom = custom;
                return maneuver_loiter;
            };

            static IMC::Loiter make_message(double lat, double lon, float z, uint16_t duration,
                                            float radius, IMC::Loiter::DirectionEnum direction) {
                return make_message(1000, lat, lon, z, IMC::ZUnits::Z_HEIGHT,
                                    duration, 17.0, IMC::SpeedUnits::SUNITS_METERS_PS,
                                    IMC::Loiter::LoiterTypeEnum::LT_CIRCULAR, radius, 1, 1, direction, "tuplelist");
            };
        };

        class SequentialPlanTransitionListFactory {
        public:
            static IMC::MessageList<IMC::PlanTransition> make_message(const std::vector<IMC::PlanManeuver>& maneuvers) {
                auto transitions = IMC::MessageList<IMC::PlanTransition>();

                for (size_t i = 0; i < maneuvers.size() - 1; ++i) {
                    auto pt = IMC::PlanTransition();
                    pt.source_man = maneuvers[i].maneuver_id;
                    pt.dest_man = maneuvers[i + 1].maneuver_id;
                    pt.conditions = "ManeuverIsDone";
                    transitions.push_back(pt);
                }
                return transitions;

            }

            static IMC::MessageList<IMC::PlanTransition>
            make_message(const IMC::MessageList<IMC::PlanManeuver>& maneuvers) {
                auto transitions = IMC::MessageList<IMC::PlanTransition>();

                for (auto it = maneuvers.begin(); it != maneuvers.end() - 1; ++it) {
                    auto pt = IMC::PlanTransition();
                    pt.source_man = (*it)->maneuver_id;
                    pt.dest_man = (*(it + 1))->maneuver_id;
                    pt.conditions = "ManeuverIsDone";
                    transitions.push_back(pt);
                }
                return transitions;

            }

        };

        class PlanSpecificationFactory {
        public:
            static IMC::PlanSpecification make_message() {

                auto man_goto0 = SAOP::neptus::GotoFactory::make_message(0.73050675, -0.11706505, 800);
                auto man_goto1 = SAOP::neptus::GotoFactory::make_message(0.73060675, -0.11706505, 800);
                auto man_goto2 = SAOP::neptus::GotoFactory::make_message(0.73060675, -0.11716505, 800);

                auto pm0 = SAOP::neptus::PlanManeuverFactory::make_message("Goto0", man_goto0);
                auto pm1 = SAOP::neptus::PlanManeuverFactory::make_message("Goto1", man_goto1);
                auto pm2 = SAOP::neptus::PlanManeuverFactory::make_message("Goto2", man_goto2);

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

            static IMC::PlanSpecification make_message(const std::string& name, std::vector<Waypoint3d> wgs_waypoints,
                                                       std::vector<std::string> maneuver_names) {
                ASSERT(wgs_waypoints.size() > 0);
                ASSERT(wgs_waypoints.size() == maneuver_names.size());

                auto pmx = IMC::MessageList<IMC::PlanManeuver>();

                for (size_t i = 0; i != wgs_waypoints.size(); ++i) {
                    auto man_gotox = SAOP::neptus::GotoFactory::make_message(
                            wgs_waypoints[i].y, wgs_waypoints[i].x, static_cast<float>(wgs_waypoints[i].z));
                    pmx.push_back(SAOP::neptus::PlanManeuverFactory::make_message(maneuver_names[i],
                                                                                  man_gotox));
                }

//                // Loiter when finished
//                auto man_loiterend = SAOP::neptus::LoiterFactory::make_message(
//                        wgs_waypoints.back().y, wgs_waypoints.back().x, static_cast<float>(wgs_waypoints.back().z), 0,
//                        200, IMC::Loiter::DirectionEnum::LD_CLOCKW);
//                pmx.push_back(SAOP::neptus::PlanManeuverFactory::make_message("Loiter", man_loiterend));

                auto plan_spec = IMC::PlanSpecification();
                plan_spec.plan_id = name;
                plan_spec.start_man_id = maneuver_names[0];
                plan_spec.maneuvers = pmx;
                plan_spec.transitions = SequentialPlanTransitionListFactory::make_message(pmx);

                return plan_spec;
            }
        };

        class PlanDBFactory {
        public:
            static IMC::PlanDB make_message() {
                auto plan_spec = SAOP::neptus::PlanSpecificationFactory::make_message();
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

            static IMC::PlanDB make_message(const std::string& name, std::vector<Waypoint3d> wgs84_waypoints) {
                std::vector<std::string> maneuver_names = {};
                for (size_t i = 0; i < wgs84_waypoints.size(); ++i){
                    maneuver_names.emplace_back(std::to_string(i));
                }
                auto plan_spec = SAOP::neptus::PlanSpecificationFactory::make_message(name, wgs84_waypoints, maneuver_names);
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

        class PlanControlFactory {
        public:
            static IMC::PlanControl make_start_plan_message(const std::string& plan_id,
                                                            uint16_t request_id = 0) {
                auto pc = IMC::PlanControl();
                pc.type = IMC::PlanControl::TypeEnum::PC_REQUEST;
                pc.op = IMC::PlanControl::OperationEnum::PC_START;
                pc.request_id = request_id;
                pc.plan_id = plan_id;
                pc.flags = 0;

                return pc;
            }

            static IMC::PlanControl make_load_plan_message(const IMC::PlanSpecification& plan_spec,
                                                           uint16_t request_id = 0) {
                IMC::PlanControl pc = IMC::PlanControl();
                pc.type = IMC::PlanControl::TypeEnum::PC_REQUEST;
                pc.op = IMC::PlanControl::OperationEnum::PC_LOAD;
                pc.request_id = request_id;
                pc.plan_id = plan_spec.plan_id;
                pc.flags = 0;
                pc.arg = IMC::InlineMessage<IMC::Message>();
                pc.arg.set(plan_spec);

                return pc;
            }
        };

        class WindSpeedFactory {
        public:
            static IMC::WindSpeed make_message(float direction, float speed) {
                auto ws = IMC::WindSpeed();
                ws.direction = direction;
                ws.speed = speed;
                ws.turbulence = 0;

                return ws;
            }
        };
    }
}

#endif //PLANNING_CPP_IMCMESSAGEFACTORY_H
