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

#include "ext/dubins.h"
#include "ext/optional.h"
#include "core/structures/waypoint.h"
#include <cmath>
#include <memory>

#undef LSL
#undef LSR
#undef RSL
#undef RSR
#undef RLR
#undef LRL

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
    SSLS,
    SLSS,
    //SSLSS, // TODO: idk if this case may exist or not.
};

enum Dubins3dGoalAltitude {
    Flat,
    Low,
    Medium,
    High,
};

/* Just the basic information to assess the cost of performing a Dubins 3D path*/
struct Dubins3dPathLength {
    // Path configuration
    Dubins3dPathType configuration = {};
    Dubins2dPathType configuration_2d = {};
    Dubins3dGoalAltitude goal_altitude = {};

    // start and end waypoints
    Waypoint3d wp_s {0,0,0,0}; // start waypoint
    Waypoint3d wp_e {0,0,0,0}; // end waypoint

    // general path parameters
    double R = 0; // radius of spirals
    double gamma = 0; // flight path angle (constant throughout maneuver)
    double L = 0; // length of Dubins path
    double L_2d = 0; // length of Dubins path in the xy plane

    Dubins3dPathLength(const Waypoint3d& from, const Waypoint3d& to, double r_min, double gamma_max) {

        ASSERT(r_min > .0);
        ASSERT(gamma_max > .0);

        /* See: Beard R.W., McLain T.W., Implementing Dubins Airplane Paths on Fixed-wing UAVs, 2013. */
        wp_s = from;
        wp_e = to;
        // We determine the configuration of the desired path: low altitude, medium altitude or high altitude
        // This needs, first, to calculate the length of the dubins 2d path
        double orig[3] = {from.x, from.y, from.dir};
        double dest[3] = {to.x, to.y, to.dir};

        if (dubins_init(orig, dest, r_min, &path2d) != EDUBOK) {
            return;
        }

        double L_path2d = dubins_path_length(&path2d);
        Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));

        double delta_z = to.z - from.z;
        double abs_delta_z = fabs(delta_z);

        if (ALMOST_EQUAL(delta_z, 0)) {
            goal_altitude = Dubins3dGoalAltitude::Flat;
            configuration = Dubins3dPathType::SLS;
            configuration_2d = type_path2d;
            L = L_path2d;
            L_2d = L_path2d;
            gamma = 0;
            R = r_min;
            return;
        }

        if (abs_delta_z <= L_path2d*tan(gamma_max)) {
            /* Low altitude case: Just perform the 2d path with a linear increment in z */
            goal_altitude = Dubins3dGoalAltitude::Low;
            configuration = Dubins3dPathType::SLS;
            configuration_2d = type_path2d;
            gamma = atan(delta_z/L_path2d);
            R = r_min;
            L = L_path2d / cos(gamma);
            L_2d = L_path2d;
            return;
        }
        if (abs_delta_z >= (L_path2d + 2 * M_PI * r_min) * tan(gamma_max)) {
            /* High altitude case */
            goal_altitude = Dubins3dGoalAltitude::High;
            k = static_cast<int>((fabs(delta_z) / tan(gamma_max) - L_path2d) / ( 2 * M_PI * r_min));
            HelixOptimizationResult optimal = helix_optimization(from, to, r_min, gamma_max, k, delta_z);
            R = optimal.R;
            path2d = optimal.path_2d;
            L_2d = (optimal.L_2d + 2 * M_PI * k * R);
            L = L_2d/cos(gamma_max);
            gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
            configuration = Dubins3dPathType::SLS;
            configuration_2d = type_path2d;
            return;
        }

        /* Medium altitude case */
        goal_altitude = Dubins3dGoalAltitude::Medium;
        R = r_min;
        gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
        configuration = gamma > 0 ? Dubins3dPathType::SSLS: Dubins3dPathType::SLSS;
        switch (type_path2d) {
            case Dubins2dPathType::LSL: //LRSL
                configuration_2d = Dubins2dPathType::RSL;
                break;
            case Dubins2dPathType::LSR: //LRSR
                configuration_2d = Dubins2dPathType::RSR;
                break;
            case Dubins2dPathType::RSL: //RLSL
                configuration_2d = Dubins2dPathType::LSL;
                break;
            case Dubins2dPathType::RSR: //RLSR
                configuration_2d = Dubins2dPathType::LSR;
                break;
            case Dubins2dPathType::RLR:
                configuration_2d = Dubins2dPathType::LRL;
                break;
            case Dubins2dPathType::LRL:
                configuration_2d = Dubins2dPathType::RLR;
                break;
            default:
                ASSERT(false);  // Something went wrong
                break;
        }
        // We don't need to call the spiral extension as we already know the desired full and xy path length.
        // We go up at full gamma_max rate.
        L_2d = fabs(delta_z/tan(gamma_max));
        L = fabs(delta_z/sin(gamma_max));
    }

    /* Get the pose in the path at distance t from the beginning */
    Waypoint3d sample(double t) {
        ASSERT(t>=0);
        ASSERT(t < L_2d);

        double new_z = wp_s.z + (wp_e.z - wp_s.z)/L_2d*t;

        double new_pos_arr[3] = {0, 0, 0};

        if (goal_altitude == Dubins3dGoalAltitude::Flat || goal_altitude == Dubins3dGoalAltitude::Low) {

            int ret = dubins_path_sample(&path2d, t, new_pos_arr);
            ASSERT(ret == 0);
            return Waypoint3d{new_pos_arr[0], new_pos_arr[1], new_z, new_pos_arr[2]};
        }

        // Cache static geometric information, so it is computed only once
        if (!geometry_cache) {
            double lambda_s = -1; // when configuration_2d is RLR, RSR or RSL
            if (configuration_2d == Dubins2dPathType::LRL || configuration_2d == Dubins2dPathType::LSL ||
                configuration_2d == Dubins2dPathType::LSR) {
                lambda_s = +1;
            }

            // Direction of the zs->cs vector
            double chi_s = wp_s.dir + M_PI_2*(lambda_s);
            // cs: center of the starting circle
            double cs_x = wp_s.x + R * cos(chi_s);
            double cs_y = wp_s.y + R * sin(chi_s);
            Waypoint3d cs = Waypoint3d{cs_x, cs_y, wp_s.z, chi_s + M_PI};

            double L_helix;
            if (goal_altitude == Dubins3dGoalAltitude::High) {
                // Length of the helix fraction of the path
                L_helix = k * 2 * M_PI * R;

            } else if (goal_altitude == Dubins3dGoalAltitude::Medium) {
                auto spiral_result = spiral_extension_beginning(wp_s, wp_e, R, gamma, lambda_s, cs, path2d);
                path2d = spiral_result.path2d;
                L_helix = spiral_result.L_ext;
            } else {
                ASSERT(false); // Bogus Dubins3dGoalAltitude
            }
            geometry_cache = std::unique_ptr<Dubins3dPathGeometry>(new Dubins3dPathGeometry(lambda_s, cs, L_helix));
        }

        if (goal_altitude == Dubins3dGoalAltitude::High || goal_altitude == Dubins3dGoalAltitude::Medium) {
            // FIXME: For now the helix is a t the beginning of the path independently of the sign of delta_z
            if (t <= geometry_cache->L_helix ) { // Helix part
                // alpha: Fraction of the helix that have been executed (in radians)
                // if t = L_helix, alpha should be 2*pi*k
                double alpha = t / R * geometry_cache->lambda_s;
                double new_pos_x = geometry_cache->cs.x + R * cos(geometry_cache->cs.dir + alpha);
                double new_pos_y = geometry_cache->cs.y + R * sin(geometry_cache->cs.dir + alpha);
                double new_dir = wp_s.dir + alpha;
                // new_dir may be > 2 * pi. This angle is wrapped to [0, 2pi] in the Waypoint3d constructor

                return Waypoint3d{new_pos_x, new_pos_y, new_z, new_dir};
            } else { // Dubins2d part
                int ret = dubins_path_sample(&path2d, t - geometry_cache->L_helix, new_pos_arr);
                ASSERT(ret == 0);
                return Waypoint3d{new_pos_arr[0], new_pos_arr[1], new_z, new_pos_arr[2]};
            }

        } else {// goal_altitude == Dubins3dGoalAltitude::Medium)
            ASSERT(false); // Not implemented yet
        }
        return {0,0,0,0};
    };

protected:
    const double MAX_LOOPS = 100;
    const double TOLERANCE = 0.01; // ~1% error

    struct HelixOptimizationResult {
        double R; // Optimal radius
        double L_2d; // Length of the resulting trajectory in the xy-plane
        DubinsPath path_2d; // Dubins 2d path using the optimal radius.
    };

    struct SpiralExtensionResult {
        Dubins3dPathType configuration_3d; // Resulting 3d configuration
        DubinsPath path2d; // Resulting path2d
        double L_ext; // Length of the extension helix
        double L_sls; // Length of the SLS part in the xy-plane
    };

    struct Dubins3dPathGeometry {
        double lambda_s = 0;
        Waypoint3d cs = {};
        double L_helix = 0;

        Dubins3dPathGeometry(double _lambda_s, Waypoint3d _cs, double _L_helix) :
                lambda_s(_lambda_s), cs(_cs), L_helix(_L_helix)
        {}
    };

    DubinsPath path2d = {};
    int k=0;
    std::unique_ptr<Dubins3dPathGeometry> geometry_cache = nullptr;

    /* Find the optimal turning radius when climbing using the bisection method */
    HelixOptimizationResult
    helix_optimization(const Waypoint3d &start, const Waypoint3d &end, double r_min, double gamma_max, int k,
                       double delta_z) const {

        DubinsPath a_path2d = {};

        double R_l = r_min; // Lowest acceptable turn radius
        double R_h = 2 * r_min; // Highest acceptable turn radius
        double R = (R_l + R_h) / 2;

        double L_2d = 0.;
        double err = 1.;
        int n_loops = 0;

        double best_r = R;
        double best_err = std::numeric_limits<double>::infinity();
        double best_L_2d = 0;
        DubinsPath best_path2d = {};

        double orig[3] = {start.x, start.y, start.dir};
        double dest[3] = {end.x, end.y, end.dir};

        while (fabs(err) > TOLERANCE) {
            int ret = dubins_init(orig, dest, R, &a_path2d);
            ASSERT(ret == 0);
            L_2d = dubins_path_length(&a_path2d);
            err = (L_2d + 2 * M_PI * k * R) * tan(fabs(gamma_max)) - fabs(delta_z);
            if (fabs(fabs(err) - fabs(best_err)) < TOLERANCE/2) {
                break;
            }

            if (fabs(err) < fabs(best_err)) {
                best_err = err;
                best_r = R;
                best_L_2d = L_2d;
                best_path2d = a_path2d;
            }

            if (err > 0) {R_h = R;} else {R_l = R;}
            R = (R_l + R_h) / 2;

            n_loops++;
            ASSERT(n_loops < MAX_LOOPS); // This loop should converge faster than MAX_LOOPS
        }
        return HelixOptimizationResult{best_r, best_L_2d, best_path2d};
    }

    SpiralExtensionResult
    spiral_extension_beginning(const Waypoint3d &start, const Waypoint3d &end, double r_min, double gamma_max,
                               double lambda_s, Waypoint3d cs, DubinsPath path2d) {
        // Parametrization of alpha based in section 4.2.3 of Beard & McLain, 2013. Case SSLS
        double alpha_l = 0; // |Lowest| acceptable alpha
        double alpha_h = 2 * M_PI * lambda_s; // |Highest| acceptable alpha
        double alpha = (alpha_l + alpha_h) / 2; // alpha is the fraction of the helix executed with respect to cs.dir

        // Extract parameters of the un-extended 2d path
        double dest[3] = {end.x, end.y, end.dir};
        double L_init = dubins_path_length(&path2d); // Initial guess for length

        double delta_z = wp_e.z-wp_s.z;

        DubinsPath path_ext; // Extension path starting after the intermediate spiral
        path_ext.type = path2d.type;
        switch ((Dubins2dPathType)path2d.type) {
            case Dubins2dPathType::LSL:
                path_ext.type = Dubins2dPathType::RSL;
                break;
            case Dubins2dPathType::LSR:
                path_ext.type = Dubins2dPathType::RSR;
                break;
            case Dubins2dPathType::RSL:
                path_ext.type = Dubins2dPathType::LSL;
                break;
            case Dubins2dPathType::RSR:
                path_ext.type = Dubins2dPathType::LSR;
                break;
            case Dubins2dPathType::RLR:
                path_ext.type = Dubins2dPathType::LRL;
                break;
            case Dubins2dPathType::LRL:
                path_ext.type = Dubins2dPathType::RLR;
                break;
            default:
            ASSERT(false);  // Something went wrong
                break;
        }

        double err = L_init - fabs(delta_z/tan(gamma_max));
        double best_err = std::numeric_limits<double>::infinity();
        double best_L_si = 0;
        double best_L_ie = 0;
        DubinsPath best_path_ext = {};
        double L_ie = 0; // Length of the part of the path after the extension
        double L_si = fabs(alpha) * r_min; //Length of the path between the starting point an the intermediate point
        int n_loops = 0;
        while (fabs(err) > TOLERANCE) {
            // Determine intermediate point between the ext. helix and the dubins2d part for this guess of alpha
            double inter_pos_x = cs.x + R * cos(cs.dir + alpha);
            double inter_pos_y = cs.y + R * sin(cs.dir + alpha);
            double inter_dir = wp_s.dir + alpha;
            L_si = fabs(alpha) * r_min;

            // Recalculate the dubins path from the intermediate point keeping the same end point
            double orig_ext[3] = {inter_pos_x, inter_pos_y, inter_dir};
            int ret = dubins_init(orig_ext, dest, r_min, &path_ext);
            ASSERT(ret == 0);
            L_ie = dubins_path_length(&path_ext);

            err = (L_ie + L_si) - fabs(delta_z/tan(gamma_max));

            if (fabs(fabs(err) - fabs(best_err)) < TOLERANCE/2) {
                break;
            }

            if (fabs(err) < fabs(best_err)) {
                best_err = err;
                best_L_si= L_si;
                best_L_ie= L_ie;
                best_path_ext = path_ext;
            }

            if (err > 0) {
                alpha_h = alpha;
            } else {
                alpha_l = alpha;
            }
            alpha = (alpha_h + alpha_l) / 2;

            n_loops++;
            ASSERT(n_loops <= MAX_LOOPS) // This loop should converge faster than MAX_LOOPS
        }

        return SpiralExtensionResult{Dubins3dPathType::SSLS, best_path_ext, best_L_si, best_L_ie};
    }
};

#endif //PLANNING_CPP_DUBINS3D_H
