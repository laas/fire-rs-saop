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

#ifndef PLANNING_CPP_SAOP_NEPTUS_H
#define PLANNING_CPP_SAOP_NEPTUS_H

#include <vector>

#include <boost/asio.hpp>

#include "../ext/coordinates.hpp"
#include "../core/waypoint.hpp"
#include "../neptus/imc_message_factories.hpp"
#include "../vns/plan.hpp"

using boost::asio::ip::udp;

namespace SAOP {

    namespace neptus {

        static uint16_t request_counter = 0; // FIXME: Not thread-safe!!

        static std::vector<Waypoint3d> WGS84_waypoints(std::vector<Waypoint3d> waypoints,
                                                       Position origin, int utm_zone = 29,
                                                       bool northern_hemisphere = true) {
            auto wgs84_wp = std::vector<Waypoint3d>();
            auto wp0 = waypoints[0];

            for (auto &wp: waypoints) {
                double lat;
                double lon;
                SAOP::ext::toWGS84(wp.y - wp0.y + origin.y, wp.x - wp0.x + origin.x,
                                   utm_zone, northern_hemisphere, &lat, &lon);
                wgs84_wp.emplace_back(Waypoint3d(lon, lat, wp.z, wp.dir));
            }

            return wgs84_wp;
        }

        class DuneLink {
        private:
            std::string ip;
            std::string port;

            size_t capacity = 1024;
        public:
            DuneLink(std::string ip, std::string port) : ip(std::move(ip)), port(std::move(port))
            {}

            void send(IMC::Message &message) {
                send(message, 3088, 25, 24290, 8);
            }

            void send(IMC::Message &message, uint16_t src, uint8_t src_ent,
                      uint16_t dst, uint8_t dst_ent) {
                try {
                    boost::asio::io_service io_service;
                    udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
                    udp::resolver resolver(io_service);
                    udp::resolver::query query(udp::v4(), ip, port);
                    udp::resolver::iterator endpoint_iterator = resolver.resolve(query);

                    IMC::ByteBuffer bb(capacity);

                    message.setSource(src);
                    message.setSourceEntity(src_ent);
                    message.setDestination(dst);
                    message.setDestinationEntity(dst_ent);
                    message.setTimeStamp();

                    IMC::Packet::serialize(&message, bb);

                    s.send_to(boost::asio::buffer(bb.getBuffer(), bb.getSize()),
                              *endpoint_iterator);
                }
                catch (std::exception &e) {
                    std::cerr << "Exception: " << e.what() << std::endl;
                }
            }
        };

        static void send_message_to_dune(const std::string &ip, const std::string &port,
                                         IMC::Message &message) {
            DuneLink dl(ip, port);
            dl.send(message);
        }

        static void send_plan_to_dune(const std::string &ip, const std::string &port, Plan plan,
                                      const std::string &plan_id, double segment_ext = 50,
                                      double sampled = -1) {
            auto ep = Trajectory(plan.trajectories()[0]).with_longer_segments(segment_ext);
            std::vector<Waypoint3d> wp;
            if (sampled <= 0) {
                wp = ep.as_waypoints();
            } else {
                wp = ep.sampled(sampled);
            }

            std::cout << "(extended) Expected duration: " << ep.duration() << std::endl;
            auto wgs84_wp = SAOP::neptus::WGS84_waypoints(wp, Position(690487, 4636304));

            auto pdb = SAOP::neptus::PlanDBFactory::make_message(plan_id, wgs84_wp);
            send_message_to_dune(ip, port, pdb);
        }

        static void send_plan_start_request(const std::string &ip, const std::string &port,
                                            const std::string &plan_id ) {
            auto pc_start = SAOP::neptus::PlanControlFactory::make_start_plan_message(
                    plan_id, request_counter++);

            DuneLink dl(ip, port);
            dl.send(pc_start);
        }

    }
}


#endif //PLANNING_CPP_SAOP_NEPTUS_H
