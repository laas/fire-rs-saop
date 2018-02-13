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

#ifndef PLANNING_CPP_DUBINSWIND_HPP
#define PLANNING_CPP_DUBINSWIND_HPP

#include <cmath>
#include <stdexcept>
#include <vector>

#include "waypoint.hpp"
#include "../ext/dubins.h"
#include "dubins3d.hpp"

namespace SAOP {

    class DubinsWindPathNotFoundException : public std::invalid_argument {
    public:
        DubinsWindPathNotFoundException(const Waypoint& from, const Waypoint& to, WindVector wind, double uav_airspeed,
                                        std::string message)
                : std::invalid_argument(
                "DubinsWindPathNotFoundException [from " + from.to_string() + " to " + to.to_string() + " airspeed " +
                std::to_string(uav_airspeed) + " wind " + wind.to_string() + "] " + message + ".") {}
    };

    class DubinsWind {
    public:

        DubinsWind(const Waypoint& from, const Waypoint& to, const WindVector& constant_wind, double uav_air_speed,
                   double turn_radius);

        std::vector<Waypoint> sampled(double l_step) const;

        std::vector<Waypoint> sampled_airframe(double l_step) const;

        Waypoint start() const {
            return wp_s;
        }

        Waypoint end() const {
            return wp_e;
        }

        double d() const {
            return d_star;
        }

        double T() const {
            return dubins_path_length(&air_path) / air_speed;
        }

        double uav_airspeed() const {
            return air_speed;
        };

        WindVector wind() const {
            return wind_vector;
        };

    private:
        // start and end waypoints
        Waypoint wp_s{0, 0, 0}; // start waypoint
        Waypoint wp_e{0, 0, 0}; // end waypoint

        // Path parameters
        WindVector wind_vector;
        double air_speed;
        double r_min; // Minimal possible radius
        double d_star; // Best d

        DubinsPath air_path = {};

        double find_d(const Waypoint& from, const Waypoint& to, WindVector wind, double uav_speed,
                      double turn_radius, DubinsPath* dubins_conf);

        opt<double> G(double d, const Waypoint& from, const Waypoint& to, WindVector wind, double uav_speed,
                      double turn_radius, DubinsPath* dubins_air_conf);

    };
}


#endif //PLANNING_CPP_DUBINSWIND_HPP
