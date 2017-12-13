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
#include "../core/structures/waypoint.hpp"
#include "../neptus/imc_message_factories.hpp"
#include "../vns/plan.hpp"

using boost::asio::ip::udp;

namespace SAOP {

    namespace neptus {
        static std::vector<Waypoint3d> WGS84_waypoints(std::vector<Waypoint3d> waypoints, Position origin,
                                                       int utm_zone = 29, bool northern_hemisphere = true) {
            auto wgs84_wp = std::vector<Waypoint3d>();
            auto wp0 = waypoints[0];

            for (auto &wp: waypoints) {
                double lat;
                double lon;
                SAOP::ext::toWGS84(wp.y - wp0.y + origin.y, wp.x - wp0.x + origin.x, utm_zone, northern_hemisphere,
                                   &lat, &lon);
                wgs84_wp.emplace_back(Waypoint3d(lon, lat, wp.z, wp.dir));
            }

            return wgs84_wp;
        }

        static void send_plan_to_dune(const std::string &ip, const std::string &port, Plan plan, std::string name,
                                      double segment_ext = 50, double sampled = -1) {
            auto ep = plan.core[0].with_longer_segments(segment_ext);
            std::vector<Waypoint3d> wp;
            if (sampled <= 0) {
                wp = ep.as_waypoints();
            } else {
                wp = ep.sampled(sampled);
            }

            std::cout << "(extended) Expected duration: " << ep.duration() << std::endl;
            auto wgs84_wp = SAOP::neptus::WGS84_waypoints(wp, Position(690487, 4636304));

            try {
                size_t max_length = 1024;

                boost::asio::io_service io_service;
                udp::socket s(io_service, udp::endpoint(udp::v4(), 0));
                udp::resolver resolver(io_service);
                udp::resolver::query query(udp::v4(), ip, port);
                udp::resolver::iterator iterator = resolver.resolve(query);

                auto pk = IMC::Packet();
                IMC::ByteBuffer bb = IMC::ByteBuffer(max_length);
                auto hbb = SAOP::neptus::PlanDBFactory::make_message(name, wgs84_wp);
                hbb.setSource(3088);
                hbb.setSourceEntity(25);
                hbb.setDestination(24290);
                hbb.setDestinationEntity(8);
                hbb.setTimeStamp();
                pk.serialize(&hbb, bb);

                std::cout << "Sent: " << hbb.toString() << std::endl;

                s.send_to(boost::asio::buffer(bb.getBuffer(), bb.getSize()), *iterator);
            }
            catch (std::exception &e) {
                std::cerr << "Exception: " << e.what() << std::endl;
            }
        }
    }
}


#endif //PLANNING_CPP_SAOP_NEPTUS_H
