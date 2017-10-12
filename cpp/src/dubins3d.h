#ifndef PLANNING_CPP_DUBINS3D_H
#define PLANNING_CPP_DUBINS3D_H

#include "ext/dubins.h"
#include "ext/optional.h"
#include "waypoint.h"
#include <cmath>

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

enum GoalAltitude {
    Flat,
    Low,
    Medium,
    High,
};

/* Just the basic information to assess the cost of performing a Dubins 3D path*/
struct Dubins3dPathLength {
    // Path configuration
    opt<Dubins3dPathType> configuration = {};
    opt<Dubins2dPathType> configuration_2d = {};
    opt<GoalAltitude> goal_altitude = {};

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
            goal_altitude = GoalAltitude::Flat;
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
            goal_altitude = GoalAltitude::Low;
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
            goal_altitude = GoalAltitude::High;
            auto k = static_cast<int>((fabs(delta_z) / tan(gamma_max) - L_path2d) / ( 2 * M_PI * r_min));
            HelixOptimizationResult optimal = helix_optimization(from, to, r_min, gamma_max, path2d, k, delta_z);
            R = optimal.R;
            L_2d = (optimal.L_2d + 2 * M_PI * k * R);
            L = L_2d/cos(gamma_max);
            gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
            configuration = Dubins3dPathType::SLS;
            configuration_2d = type_path2d;
            return;
        }

        /* Medium altitude case */
        goal_altitude = GoalAltitude::Medium;
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
                // TODO: verify this is correct. (Originally not present in Beard 2013)
                configuration_2d = Dubins2dPathType::LRL;
                break;
            case Dubins2dPathType::LRL:
                // TODO: verify this is correct. (Originally not present in Beard 2013)
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

protected:

    DubinsPath path2d = {};

    const double MAX_LOOPS = 100;
    const double TOLERANCE = 0.1;

    struct HelixOptimizationResult {
        double R; // Optimal radius
        double L_2d; // Length of the resulting trajectory in the xy-plane
    };

    /* Find the optimal turning radius when climbing using the bisection method */
    HelixOptimizationResult
    helix_optimization(const Waypoint3d &start, const Waypoint3d &end, double r_min, double gamma_max,
                       DubinsPath path2d, int k, double delta_z) {
        double R_l = r_min; // Lowest acceptable turn radius
        double R_h = 2 * r_min; // Highest acceptable turn radius
        double R = (R_l + R_h) / 2;
        double L_2d = 0.;
        double err = 1.;
        int n_loops = 0;
        while (fabs(err) > TOLERANCE) {
            double orig[3] = {start.x, start.y, start.dir};
            double dest[3] = {end.x, end.y, end.dir};
            int ret = dubins_init(orig, dest, r_min, &path2d); // As path2d has the type already set, this call is fast
            ASSERT(ret == 0);
            L_2d = dubins_path_length(&path2d);
            err = (L_2d + 2 * M_PI * k * R) * tan(gamma_max) - fabs(delta_z);
            if (err > 0) {R_h = R;} else {R_l = R;}
            R = (R_l + R_h) / 2;

            n_loops++;
            ASSERT(n_loops < MAX_LOOPS); // This loop should converge faster than MAX_LOOPS
        }
        return HelixOptimizationResult{R, L_2d};
    }
};

/* All the information needed to define a Dubins 3D path*/
struct Dubins3dPath : public Dubins3dPathLength {

//    // start spiral
//    Position3d c_s {0,0,0}; // center of start spiral
//    double chi_s = 0; // angle of start spiral
//    int lam_s = 0; // direction of start spiral
//    int k_s = 0; // number of full start spirals
//
//    // intermediate-start spiral
//    Position3d c_si {0,0,0}; // center of intermediate-start spiral
//    double chi_si = 0; // angle of intermediate-start spiral
//    int lam_si = 0; // direction of intermediate-start spiral
//    int k_si = 0; // number of full intermediate-start spirals
//
//    //intermediate-end spiral
//    Position3d c_ei {0,0,0}; // center of intermediate-end spiral
//    double chi_ei = 0; // angle of intermediate-end spiral
//    int lam_ei = 0; // direction of intermediate-end spiral
//    int k_ei = 0; // number of full intermediate-end spirals
//
//    // end spiral
//    Position3d c_e {0,0,0}; // center of end spiral
//    double chi_e = 0; // angle of end spiral
//    int lam_e = 0; // direction of end spiral
//    int k_e = 0; // number of full end spirals
//
//    // hyperplane H_s: switch from first spiral to next segment
//    Position3d w_s {0,0,0}; // center of hyperplane
//    Position3d q_s {0,0,0}; // normal vector to hyperplane
//
//    // hyperplane H_si: switch from second start spiral to next segment
//    Position3d w_si {0,0,0}; // center of hyperplane
//    Position3d q_si {0,0,0}; // normal vector to hyperplane
//
//    // hyperplane H_l: switch from straight-line to next segment
//    Position3d w_l {0,0,0}; // center of hyperplane
//    Position3d q_l {0,0,0}; // normal vector to hyperplane
//
//    // hyperplane H_ei: switch from second end spiral
//    Position3d w_ei {0,0,0}; // center of hyperplane
//    Position3d q_ei {0,0,0}; // normal vector to hyperplane
//
//    // hyperplane H_e: end of Dubins path
//    Position3d w_e {0,0,0}; // center of hyperplane
//    Position3d q_e {0,0,0}; // normal vector to hyperplane

    Dubins3dPath(const Waypoint3d& from, const Waypoint3d& to, double r_min, double gamma_max)
            : Dubins3dPathLength (from, to, r_min, gamma_max) {
//        compute_geometry();
    }

    explicit Dubins3dPath(Dubins3dPathLength &dubins3dPathLength) : Dubins3dPathLength(dubins3dPathLength) {
//        compute_geometry();
    }

//    Position3d path_sample(double t) {
//        if (configuration_2d || configuration || goal_altitude) {
//            double L_path2d = dubins_path_length(&path2d);
//            Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));
//
//            double delta_z = wp_e.z - wp_s.z;
//            double abs_delta_z = fabs(delta_z);
//
//            if (*goal_altitude == GoalAltitude::Flat) {
//
//            }
//        } else {
//            return {0,0,0};
//        }
//    }
//
//    /* See: Beard R.W., McLain T.W., Implementing Dubins Airplane Paths on Fixed-wing UAVs, 2013. */
//    void compute_geometry() {
////        wp_s = from;
////        wp_e = to;
//        // We determine the configuration of the desired path: low altitude, medium altitude or high altitude
//        // This needs, first, to calculate the length of the dubins 2d path
//        DubinsPath path2d;
//        double orig[3] = {wp_s.x, wp_s.y, wp_s.dir};
//        double dest[3] = {wp_e.x, wp_e.y, wp_e.dir};
//
//        double L_path2d = dubins_path_length(&path2d);
//        Dubins2dPathType type_path2d = static_cast<Dubins2dPathType>(dubins_path_type(&path2d));
//
//        double delta_z = wp_e.z - wp_s.z;
//        double abs_delta_z = fabs(delta_z);
//
//        if (!configuration_2d || !configuration || !goal_altitude) {
//            return;
//        }
//
//        if (*goal_altitude == GoalAltitude::Flat) {
//
//        }
//
//        switch (*configuration) {
//            case Dubins3dPathType::SLS: // Flat, Low or High
//                int k = 0;
//                if (*goal_altitude == GoalAltitude::High) {
//                    // FIXME: gamma gives the same as gamma_max? R gives the same as r_min?
//                    k = static_cast<int>((abs_delta_z / tan(gamma) - L_path2d) / (2 * M_PI * R));
//                    if (delta_z>=0) {
//                        k_s = k, k_e = 0; // k spirals at the beginning
//                    } else {
//                        k_s = 0; k_e = k; // k spirals at the end
//                    }
//                }
//                switch (*configuration_2d) {
//                    case Dubins2dPathType::LSL:
//                        // Start spiral
//                        c_s = {0,0,0}; // center of start spiral
//                        chi_s = 0; // angle of start spiral //FIXME
//                        lam_s = 1; // direction of start spiral
//
//                        // End spiral
//                        c_e = {0,0,0}; // center of end spiral
//                        chi_e = 0; // angle of end spiral  //FIXME
//                        lam_e = 1; // direction of end spiral
//                        // H_s hyperplane
//                        // H_l hyperplane
//                        // H_e hyperplane
//                        break;
//                    default:
//                        ASSERT(false);  // Something went wrong
//                        break;
//                }
//
//                // zs: starting position
//                double zs_x = wp_s.x;
//                double zs_y = wp_s.y;
//                // Starting angle for the first spiral that is perpendicular to UAV heading
//                double chi_s = wp_s.dir + M_PI_2*(-lam_s);
//                // cs: center of the starting circle
//                double cs_x = zs_x + R * (cos(M_PI_2*lam_s) * cos(wp_s.dir) - sin(M_PI_2*lam_s) * sin(wp_s.dir));
//                double cs_y = zs_y + R * (sin(M_PI_2*lam_s) * cos(wp_s.dir) + cos(M_PI_2*lam_s) * sin(wp_s.dir));
//                c_s = {cs_x, cs_y, };
//                break;
//            case Dubins3dPathType::SSLS:
//                configuration_2d = Dubins2dPathType::RSR;
//                lam_si = +1;
//                lam_s  = -1;
//                lam_e  = -1;
//                lam_ei = 0;
//                break;
//            case Dubins3dPathType::SLSS:
//                configuration_2d = Dubins2dPathType::LSL;
//                lam_si = -1;
//                lam_s  = +1;
//                lam_e  = +1;
//                lam_ei = 0;
//                break;
//            default:
//                ASSERT(false);  // Something went wrong
//                break;
//        }
//
//        if (this->goal_altitude == GoalAltitude::Flat) {
//            goal_altitude = GoalAltitude::Flat;
//            L = L_path2d;
//            L_2d = L_path2d;
//            gamma = 0;
//            configuration = Dubins3dPathType::SLS;
//            configuration_2d = type_path2d;
//            R = r_min;
//            return L;
//        } else if (this->goal_altitude == GoalAltitude::Low) {
//            /* Low altitude case: Just perform the 2d path with a linear increment in z */
//            goal_altitude = GoalAltitude::Low;
//            gamma = atan(delta_z/L_path2d);
//            configuration = Dubins3dPathType::SLS;
//            R = r_min;
//            L = L_path2d / cos(gamma);
//            L_2d = L_path2d;
//
//        } else if (this->goal_altitude == GoalAltitude::High) {
//            /* High altitude case */
//            goal_altitude = GoalAltitude::High;
//            auto k = static_cast<int>((fabs(delta_z) / tan(gamma_max) - L_path2d) / ( 2 * M_PI * r_min));
//            if (delta_z>=0) {
//                k_s = k, k_e = 0; // k spirals at the beginning
//            } else {
//                k_s = 0; k_e = k; // k spirals at the end
//            }
//
//            HelixOptimizationResult optimal = helix_optimization(from, to, r_min, gamma_max, path2d, k, delta_z);
//            R = optimal.R;
//            L_2d = (optimal.L_2d + 2 * M_PI * k * R);
//            L = L_2d/cos(gamma_max);
//            configuration = Dubins3dPathType::SLS;
//            configuration_2d = type_path2d;
//            gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
//
//        } else { /* this->goal_altitude == GoalAltitude::Medium */
//            /* Medium altitude case */
//            if (abs_delta_z > 0) {
//                goal_altitude = GoalAltitude::Medium;
//                gamma = gamma_max;
//                configuration = Dubins3dPathType::SSLS;
//                switch (type_path2d) {
//                    case Dubins2dPathType::LSL: //LRSL
//                        configuration_2d = Dubins2dPathType::RSL;
//                        lam_si = +1;
//                        lam_s  = -1;
//                        lam_e  = +1;
//                        lam_ei = 0;
//                        break;
//                    case Dubins2dPathType::LSR: //LRSR
//                        configuration_2d = Dubins2dPathType::RSR;
//                        lam_si = +1;
//                        lam_s  = -1;
//                        lam_e  = -1;
//                        lam_ei = 0;
//                        break;
//                    case Dubins2dPathType::RSL: //RLSL
//                        configuration_2d = Dubins2dPathType::LSL;
//                        lam_si = -1;
//                        lam_s  = +1;
//                        lam_e  = +1;
//                        lam_ei = 0;
//                        break;
//                    case Dubins2dPathType::RSR: //RLSR
//                        configuration_2d = Dubins2dPathType::LSR;
//                        lam_si = -1;
//                        lam_s  = +1;
//                        lam_e  = -1;
//                        lam_ei = 0;
//                        break;
//                    case Dubins2dPathType::RLR:
//                    ASSERT(false);  // FIXME: Idk how to treat it
//                        break;
//                    case Dubins2dPathType::LRL:
//                    ASSERT(false);  // FIXME: Idk how to treat it
//                        break;
//                    default:
//                    ASSERT(false);  // Something went wrong
//                        break;
//                }
//                double expected_L = fabs(delta_z/tan(gamma_max));
////            opt<SpiralExtensionResult> res = spiral_extension_beginning(from, to, r_min, gamma_max, path2d, delta_z);
////            if (res) {
////                ASSERT(res->configuration_3d == Dubins3dPathType::SSLS);
////                L_2d = res->L_sls + res->L_i;
////                L = L_2d / cos(gamma_max);
////                double spiral_extension_error = L - expected_L;
////            }
//            } else {
//                gamma = -gamma_max;
//                configuration = Dubins3dPathType::SLSS;
//                //FIXME: NOT IMPLEMENTED ADD_HELIX_AT_END
//            }
//        }
//    }


private:

    struct SpiralExtensionResult {
        double configuration_3d; //
        double L_i; // Length of the intermediate spiral in the xy-plane
        double L_sls; // Length of the SLS part in the xy-plane
        double ci_x; // Center (x) of intermediate-start (or intermediate-end) spiral
        double ci_y; // Center (y) of intermediate-start (or intermediate-end) spiral
        double chi_i; // Angle of intermediate-start spiral
    };


    opt<SpiralExtensionResult>
    spiral_extension_beginning(const Waypoint3d &start, const Waypoint3d &end, double r_min, double gamma_max,
                               DubinsPath path2d, double delta_z) {
        // Parametrization of φ as explained in section 4.2.3 of Beard & McLain, 2013. Case SSLS
        double chi_l = 0; // Lowest acceptable φ
        double chi_h = 2 * M_PI; // Highest acceptable φ
        double chi = (chi_l + chi_h) / 2; // φ is the arc of the circle with respect to φ_s

        // Extract parameters of the un-extended 2d path
        double dest[3] = {end.x, end.y, end.dir};
        double L_init = dubins_path_length(&path2d);
        double lambda_i = 1; // First turn to the left
        int path2d_type = dubins_path_type(&path2d);
        if (path2d_type == RSL || path2d_type == RSR || path2d_type == RLR) {
            lambda_i = -1; /* First turn to the right*/
        }

        // Find the zi point. See fig. 10 of Beard&McLain 2013 for details.
        // zs: starting position
        double zs_x = start.x;
        double zs_y = start.y;
        // cs: center of the starting circle

        double chi_s = start.dir + M_PI_2*(-lambda_i); // Starting angle for the first spiral that is perpendicular to UAV heading

        double cs_x = zs_x + r_min * (cos(M_PI_2*lambda_i) * cos(start.dir) - sin(M_PI_2*lambda_i) * sin(start.dir));
        double cs_y = zs_y + r_min * (sin(M_PI_2*lambda_i) * cos(start.dir) + cos(M_PI_2*lambda_i) * sin(start.dir));

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
            ASSERT(false);  // FIXME: Idk how to treat it
                break;
            case Dubins2dPathType::LRL:
            ASSERT(false);  // FIXME: Idk how to treat it
                break;
            default:
            ASSERT(false);  // Something went wrong
                break;
        }

        double err = L_init - fabs(delta_z/tan(gamma_max));
        double L_ie = 0; // Length of the part of the path after the extension
        double L_si = fabs(chi) * r_min; //Length of the path between the starting point an the intermediate point
        int n_loops = 0;
        while (fabs(err) > TOLERANCE) {
            // Determine intermediate point for guessed φ
            double zi_x = cs_x + cos(lambda_i * (chi)) * (zs_x-cs_x) - sin(lambda_i * (chi)) * (zs_y-cs_y);
            double zi_y = cs_y + sin(lambda_i * (chi)) * (zs_x-cs_x) + cos(lambda_i * (chi)) * (zs_y-cs_y);
            double chi_i = chi_s + chi;
            L_si = fabs(chi) * r_min;

            // Recalculate the dubins path from the intermediate point keeping the same end point
            double orig_ext[3] = {zi_x, zi_y, chi_i - M_PI_2*(-lambda_i)};
            int ret = dubins_init(orig_ext, dest, r_min, &path_ext);
            ASSERT(ret == 0);

            L_ie = dubins_path_length(&path_ext); // We could speedup this function by using the DubinsType

            err = (L_ie + L_si) - fabs(delta_z/tan(gamma_max));
            if (err > 0) {
                chi_h = chi;
            } else {
                chi_l = chi;
            }
            chi = (chi_h + chi_l) / 2;

            n_loops++;
            if(n_loops >= MAX_LOOPS) { ASSERT(false); } // This loop should converge faster than MAX_LOOPS
        }

        return SpiralExtensionResult{Dubins3dPathType::SSLS, L_si, L_ie, cs_x, cs_y, chi};

    }

    opt<SpiralExtensionResult>
    spiral_extension_end(const Waypoint3d &start, const Waypoint3d &end, double r_min, double gamma_max,
                               DubinsPath path2d, double delta_z) {

        // FIXME: IMPLEMENT ME!
        return {};
    }

};

#endif //PLANNING_CPP_DUBINS3D_H
