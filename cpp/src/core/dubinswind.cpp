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

#include "dubinswind.hpp"
#include "../ext/dubins.h"

namespace SAOP {

    DubinsWind::DubinsWind(const Waypoint& from, const Waypoint& to, const WindVector &constant_wind, double uav_air_speed,
                           double turn_radius) {

        wp_s = from;
        wp_e = to;
        r_min = turn_radius;
        wind_vector = constant_wind;
        air_speed = uav_air_speed;

        if (from.as_point().dist(to.as_point()) <= 2 * r_min) {
            throw DubinsWindPathNotFoundException(from, to, wind_vector, turn_radius, "wp are closer than 4*r_min");
        }

        // We only accept CSC traj types for now
        auto dubins_types = std::vector<Dubins2dPathType> {Dubins2dPathType::LSL,
                                                           Dubins2dPathType::LSR,
                                                           Dubins2dPathType::RSL,
                                                           Dubins2dPathType::RSR};
        // d* is chosen among these d values that are going to be computed
        auto dd = std::vector<double>(dubins_types.size(), std::numeric_limits<double>::infinity());
        d_star = std::numeric_limits<double>::infinity();

        // Finding d for each dubins type == finding d for each piece of G(d)
        for (size_t i = 0ul; i < dubins_types.size(); ++i) {
            // Find rdv between UAV dubins path in air frame and the target point
            DubinsPath a_path;
            a_path.type = dubins_types[i];
            dd[i] = find_d(wp_s, wp_e, wind_vector, air_speed, turn_radius, &a_path);
            if (dd[i] < d_star) {
                path_air = a_path;
                d_star = dd[i];
            }
        }

        if (path_air.type == -1) {
           throw DubinsWindPathNotFoundException(from, to, wind_vector, turn_radius, "d* couldn't be found");
        }
    }

    double DubinsWind::find_d(const Waypoint& from, const Waypoint& to, WindVector wind, double uav_speed,
                              double turn_radius, DubinsPath* dubins_air_conf) {
        double da = 0.; // G(o) > 0 (Remark 2)
        double db = 10000; // lim d→∞ (G(d)) < 0 (Remark 2)
        // G(da) must be positive
        // G(db) must be negative

        ASSERT((dubins_air_conf->type >= 0) && (dubins_air_conf->type <= 5));

        double max_iterations = 100;
        double epsilon = 0.1;

        double n = 0;

        while (n < max_iterations) {
            auto dc = (da + db) / 2; // Set new midpoint
            auto to_air = to.move(dc, -wind.dir());
            auto t_vt = dc * wind.speed();
            double orig_air[3] = {from.x, from.y, from.dir};
            double dest_air[3] = {to_air.x, to_air.y, to_air.dir};
            auto ret = dubins_init_with_type(orig_air, dest_air, turn_radius, dubins_air_conf, dubins_air_conf->type);
            if (ret != EDUBOK) {
                // TODO: Path does not exist! What to do? idk :-(
                return std::numeric_limits<double>::infinity(); // Solution not found
            } else {
                auto g = dubins_path_length(dubins_air_conf) / uav_speed - t_vt;
                if (fabs(g) < epsilon || ((db - da) / 2) < epsilon) {
                    return dc;
                }
                if (g > 0) { da = dc; } else { db = dc; } // G(da) must be positive, G(db) must be negative
            }
            ++n;
        }

        return std::numeric_limits<double>::infinity(); // Solution not found
    }
}