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

#include "dubins3d.hpp"


namespace SAOP {

    Dubins3dPathLength::Dubins3dPathLength(const Waypoint3d& from, const Waypoint3d& to, double _r_min,
                                           double gamma_max) {
        ASSERT(_r_min > .0);
        ASSERT(gamma_max > .0);

        R_min = _r_min;

        /* See: Beard R.W., McLain T.W., Implementing Dubins Airplane Paths on Fixed-wing UAVs, 2013. */
        wp_s = from;
        wp_e = to;
        // We determine the configuration of the desired path: low altitude, medium altitude or high altitude
        // This needs, first, to calculate the length of the dubins 2d path
        double orig[3] = {from.x, from.y, from.dir};
        double dest[3] = {to.x, to.y, to.dir};

        int ret = dubins_init(orig, dest, R_min, &path2d);
        ASSERT(ret == EDUBOK);
        double L_path2d = dubins_path_length(&path2d);

        double delta_z = to.z - from.z;
        double abs_delta_z = fabs(delta_z);

        if (ALMOST_EQUAL(delta_z, 0)) {
            goal_altitude = Dubins3dGoalAltitude::Flat;
            gamma = 0;
            L = L_path2d;
            L_2d = L_path2d;
            return;
        }

        if (abs_delta_z <= L_path2d * tan(gamma_max)) {
            /* Low altitude case: Just perform the 2d path with a linear increment in z */
            goal_altitude = Dubins3dGoalAltitude::Low;
            gamma = atan2(delta_z, L_path2d);
            L = L_path2d / cos(gamma);
            L_2d = L_path2d;
            return;
        }
        if (abs_delta_z >= (L_path2d + 2 * M_PI * R_min) * tan(gamma_max)) {
            /* High altitude case */
            goal_altitude = Dubins3dGoalAltitude::High;
            gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
            L_2d = fabs(delta_z / tan(gamma_max));
            L = fabs(L_2d / cos(gamma_max));
            return;
        }

        /* Medium altitude case */
        goal_altitude = Dubins3dGoalAltitude::Medium;
        gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
        // We don't need to call the spiral extension as we already know the desired full and xy path length.
        // We go up at full gamma_max rate.
        L_2d = fabs(delta_z / tan(gamma_max));
        L = fabs(L_2d / cos(gamma_max));
    }

    Dubins3dPath::Dubins3dPath(const Waypoint3d& from, const Waypoint3d& to, double r_min, double gamma_max) :
            Dubins3dPathLength(from, to, r_min, gamma_max) {

        if (goal_altitude == Dubins3dGoalAltitude::Flat || goal_altitude == Dubins3dGoalAltitude::Low) {
            Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));
            if (type_path2d == Dubins2dPathType::RSR || type_path2d == Dubins2dPathType::LRL) {
                configuration = Dubins3dPathType::SSS;
            } else {
                configuration = Dubins3dPathType::SLS;
            }
            configuration_2d = type_path2d;
            R = r_min;
            unprecise_L_2d = L_2d;
        } else if (goal_altitude == Dubins3dGoalAltitude::High) {
            double delta_z = wp_e.z - wp_s.z;
            k = static_cast<int>((fabs(delta_z) / tan(gamma_max) - dubins_path_length(&path2d)) /
                                 (2 * M_PI * r_min));
            HelixOptimizationResult optimal = helix_optimization(from, to, r_min, gamma_max, k, delta_z);
            R = optimal.R;
            path2d = optimal.path_2d;
            L_helix = k * 2 * M_PI * R;
            unprecise_L_2d = optimal.L_2d + L_helix;

            // Get lambda_s
            lambda_s = -1; // when configuration_2d is RLR, RSR or RSL
            if (configuration_2d == Dubins2dPathType::LRL || configuration_2d == Dubins2dPathType::LSL ||
                configuration_2d == Dubins2dPathType::LSR) {
                lambda_s = +1;
            }

            //Get cs
            double chi_s = wp_s.dir + M_PI_2 * (lambda_s); // Direction of the zs->cs vector
            double cs_x = wp_s.x + R * cos(chi_s);
            double cs_y = wp_s.y + R * sin(chi_s);
            cs = Waypoint3d{cs_x, cs_y, wp_s.z, chi_s + M_PI};

            Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));
            if (type_path2d == Dubins2dPathType::RSR || type_path2d == Dubins2dPathType::LRL) {
                configuration = Dubins3dPathType::SSS;
            } else {
                configuration = Dubins3dPathType::SLS;
            }
            configuration_2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));

        } else { //goal_altitude == Dubins3dGoalAltitude::Medium
            R = r_min;

            Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));
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

            lambda_s = -1; // when configuration_2d is RLR, RSR or RSL
            if (configuration_2d == Dubins2dPathType::LRL || configuration_2d == Dubins2dPathType::LSL ||
                configuration_2d == Dubins2dPathType::LSR) {
                lambda_s = +1;
            }

            // Direction of the zs->cs vector
            double chi_s = wp_s.dir + M_PI_2 * (lambda_s);
            // cs: center of the starting circle
            double cs_x = wp_s.x + R * cos(chi_s);
            double cs_y = wp_s.y + R * sin(chi_s);
            cs = Waypoint3d{cs_x, cs_y, wp_s.z, chi_s + M_PI};

            auto spiral_result = spiral_extension_beginning(wp_s, wp_e, R, gamma, lambda_s, cs, path2d);
            path2d = spiral_result.path2d;
            unprecise_L_2d = spiral_result.L_sls + spiral_result.L_ext;
            L_helix = spiral_result.L_ext;

            //Note the spiral is always added to the beginning
            if (type_path2d == Dubins2dPathType::RSR || type_path2d == Dubins2dPathType::LRL) {
                configuration = Dubins3dPathType::SSSS;
            } else {
                configuration = Dubins3dPathType::SSLS;
            }
        }
    }

    /* Get the pose in the path at distance t from the beginning */
    Waypoint3d Dubins3dPath::sample(double t) {
        ASSERT(t >= 0);
        ASSERT(t < L_2d);

        /* Scale t to unprecise_L_2d. If helix_optimization or spiral_extension don't find the exact
         * value, unprecise_L_2d and L_2d will differ a bit. */
        t = t / L_2d * unprecise_L_2d;

        double new_z = wp_s.z + (wp_e.z - wp_s.z) / L_2d * t;
        double new_pos_arr[3] = {0, 0, 0};

        if (goal_altitude == Dubins3dGoalAltitude::Flat || goal_altitude == Dubins3dGoalAltitude::Low) {
            int ret = dubins_path_sample(&path2d, t, new_pos_arr);
            ASSERT(ret == 0);
            return Waypoint3d{new_pos_arr[0], new_pos_arr[1], new_z, new_pos_arr[2]};
        }

        if (goal_altitude == Dubins3dGoalAltitude::High || goal_altitude == Dubins3dGoalAltitude::Medium) {
            // NOTE: For now the helix is a t the beginning of the path independently of the sign of delta_z
            if (t <= L_helix) { // Helix/spiral part
                // alpha: Fraction of the helix that have been executed (in radians)
                // if t = L_helix, alpha should be 2*pi*k
                double alpha = t / R * lambda_s;
                double new_pos_x = cs.x + R * cos(cs.dir + alpha);
                double new_pos_y = cs.y + R * sin(cs.dir + alpha);
                double new_dir = wp_s.dir + alpha;
                // new_dir may be > 2 * pi. This angle is wrapped to [0, 2pi] in the Waypoint3d constructor
                return Waypoint3d{new_pos_x, new_pos_y, new_z, new_dir};
            } else { // Dubins2d part
                int ret = dubins_path_sample(&path2d, t - L_helix, new_pos_arr);
                ASSERT(ret == EDUBOK);
                return Waypoint3d{new_pos_arr[0], new_pos_arr[1], new_z, new_pos_arr[2]};
            }

        } else {
            ASSERT(false); // Not implemented yet
        }
        return {0, 0, 0, 0};
    }

    /* Find the optimal turning radius when climbing using the bisection method */
    Dubins3dPath::HelixOptimizationResult
    Dubins3dPath::helix_optimization(const Waypoint3d& start, const Waypoint3d& end, double r_min, double gamma_max,
                                     int k, double delta_z) const {
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
            if (fabs(fabs(err) - fabs(best_err)) < TOLERANCE / 2) {
                break;
            }

            if (fabs(err) < fabs(best_err)) {
                best_err = err;
                best_r = R;
                best_L_2d = L_2d;
                best_path2d = a_path2d;
            }

            if (err > 0) { R_h = R; } else { R_l = R; }
            R = (R_l + R_h) / 2;

            n_loops++;
            ASSERT(n_loops < MAX_LOOPS);
            if (n_loops > MAX_LOOPS) {
                std::cerr << "WARNING: Helix optimization do not converge.\tn_loops = " << n_loops
                          << "\tz err:" << err << std::endl;
                break;
            }
        }
        return HelixOptimizationResult{best_path2d, best_r, best_L_2d};
    }

    Dubins3dPath::SpiralExtensionResult
    Dubins3dPath::spiral_extension_beginning(const Waypoint3d& start, const Waypoint3d& end, double r_min,
                                             double gamma_max, double lambda_s, Waypoint3d cs, DubinsPath path2d) {
        // Parametrization of alpha based in section 4.2.3 of Beard & McLain, 2013. Case SSLS
        double alpha_l = 0; // |Lowest| acceptable alpha
        double alpha_h = 2 * M_PI * lambda_s; // |Highest| acceptable alpha
        double alpha =
                (alpha_l + alpha_h) / 2; // alpha is the fraction of the helix executed with respect to cs.dir

        // Extract parameters of the un-extended 2d path
        double dest[3] = {end.x, end.y, end.dir};
        double L_init = dubins_path_length(&path2d); // Initial guess for length

        double delta_z = wp_e.z - wp_s.z;

        DubinsPath path_ext; // Extension path starting after the intermediate spiral
        path_ext.type = path2d.type;
        switch ((Dubins2dPathType) path2d.type) {
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

        double err = L_init - fabs(delta_z / tan(gamma_max));
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

            err = (L_ie + L_si) - fabs(delta_z / tan(gamma_max));

            if (fabs(fabs(err) - fabs(best_err)) < TOLERANCE / 2) {
                break;
            }

            if (fabs(err) < fabs(best_err)) {
                best_err = err;
                best_L_si = L_si;
                best_L_ie = L_ie;
                best_path_ext = path_ext;
            }

            if (err > 0) {
                alpha_h = alpha;
            } else {
                alpha_l = alpha;
            }
            alpha = (alpha_h + alpha_l) / 2;

            n_loops++;
            ASSERT(n_loops < MAX_LOOPS);
            if (n_loops > MAX_LOOPS) {
                std::cerr << "WARNING: Spiral extension do not converge. n_loops = " << n_loops
                          << std::endl; // This loop should converge faster than MAX_LOOPS
                break;
            }
        }
        return SpiralExtensionResult{best_path_ext, best_L_si, best_L_ie};
    }
}