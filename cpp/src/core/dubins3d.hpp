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

#ifndef PLANNING_CPP_DUBINS3D_H
#define PLANNING_CPP_DUBINS3D_H

#include "../ext/dubins.h"
#include "../ext/optional.hpp"
#include "waypoint.hpp"
#include <cmath>
#include <limits>
#include <memory>
#include <iostream>

#undef LSL
#undef LSR
#undef RSL
#undef RSR
#undef RLR
#undef LRL

using namespace SAOP;

namespace SAOP {

    enum Dubins2dPathType {
        /* Enum values compatible with those defined in dubins.h */
        /* L = Turn left, S = Straight line, R = Turn right */
                LSL = 0,
        LSR = 1,
        RSL = 2,
        RSR = 3,
        RLR = 4,
        LRL = 5,
    };

    enum Dubins3dPathType {
        /* S = Spiral, L = Line */
                SLS = 1,
        SSS,
        SSLS,
        SLSS,
        SSSS,
    };

    enum Dubins3dGoalAltitude {
        Flat,
        Low,
        Medium,
        High,
    };

/* Just the basic information to assess the cost of performing a Dubins 3D path*/
    struct Dubins3dPathLength {

        Dubins3dGoalAltitude goal_altitude = {};

        // start and end waypoints
        Waypoint3d wp_s{0, 0, 0, 0}; // start waypoint
        Waypoint3d wp_e{0, 0, 0, 0}; // end waypoint

        // general path parameters
        double gamma = 0; // flight path angle (constant throughout maneuver)
        double L = 0; // length of Dubins path
        double L_2d = 0; // length of Dubins path in the xy plane

        Dubins3dPathLength(const Waypoint3d& from, const Waypoint3d& to, double _r_min, double gamma_max);

    protected:
        DubinsPath path2d = {};
        double R_min = 0; // Minimal possible radius
    };

    struct Dubins3dPath : public Dubins3dPathLength {

        // Path configuration
        Dubins3dPathType configuration = {};
        Dubins2dPathType configuration_2d = {};

        double R = 0; // radius of spirals
        int k = 0; // Number of full turns in the helix

        Dubins3dPath(const Waypoint3d& from, const Waypoint3d& to, double r_min, double gamma_max) ;

        /* Get the pose in the path at distance t from the beginning */
        Waypoint3d sample(double t);

    protected:
        const double MAX_LOOPS = 100;
        const double TOLERANCE = 0.01; // ~1% error

        double unprecise_L_2d = 0; // L_2d obtained after helix_optimization or spiral_extension
        double L_helix = 0; // Length of the extension helix/spiral
        int lambda_s = 0; // Rotation direction of the extension helix/spiral
        Waypoint3d cs = {}; // center of the extension helix/spiral

        struct HelixOptimizationResult {
            DubinsPath path_2d; // Dubins 2d path using the optimal radius.
            double R; // Optimal radius
            double L_2d; // Length of the resulting trajectory in the xy-plane
        };

        struct SpiralExtensionResult {
            DubinsPath path2d; // Resulting path2d
            double L_ext; // Length of the extension helix
            double L_sls; // Length of the SLS part in the xy-plane
        };

        /* Find the optimal turning radius when climbing using the bisection method */
        HelixOptimizationResult
        helix_optimization(const Waypoint3d& start, const Waypoint3d& end, double r_min, double gamma_max, int k,
                           double delta_z) const;

        SpiralExtensionResult
        spiral_extension_beginning(const Waypoint3d& start, const Waypoint3d& end, double r_min, double gamma_max,
                                   double lambda_s, Waypoint3d cs, DubinsPath path2d);
    };
}
#endif //PLANNING_CPP_DUBINS3D_H
