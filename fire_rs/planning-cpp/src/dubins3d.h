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

enum Dubins2dPathConfiguration {
    /*Enum values compatible with those defined in dubins.h*/
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5,
};

enum Dubins3dPathConfiguration {
    /* S = spiral, L = Line */
    SLS,
    SSLS,
    SLSS,
    SSLSS,
};

/* All the information needed to define a Dubins 3D path*/
struct Dubins3dPath {
    // Path configuration
    Dubins3dPathConfiguration configuration;

    // start and end waypoints
    Waypoint3d wp_s; // start position
    Waypoint3d wp_e; // end position

    // general path parameters
    double R; // radius of spirals
    double gamma; // flight path angle (constant throughout maneuver)
    double L; // length of Dubins path

    // start spiral
    Waypoint3d c_s; // center of start spiral
    // double psi_s; // angle of start spiral
    int lam_s; // direction of start spiral
    int k_s; // number of full start spirals

    // intermediate-start spiral
    Waypoint3d c_si; // center of intermediate-start spiral
    // psi_si ; // angle of intermediate-start spiral
    int lam_si; // direction of intermediate-start spiral
    int k_si; // number of full intermediate-start spirals

    //intermediate-end spiral
    Waypoint3d c_ei; // center of intermediate-end spiral
    //psi_ei; // angle of intermediate-end spiral
    int lam_ei; // direction of intermediate-end spiral
    int k_ei; // number of full intermediate-end spirals

    // end spiral
    Waypoint3d c_e; // center of end spiral
    //psi_e; // angle of end spiral
    int lam_e; // direction of end spiral
    int k_e; // number of full end spirals

    // hyperplane H_s: switch from first spiral to next segment
    Position3d w_s; // center of hyperplane
    Position3d q_s; // normal vector to hyperplane

    // hyperplane H_si: switch from second start spiral to next segment
    Position3d w_si; // center of hyperplane
    Position3d q_si; // normal vector to hyperplane

    // hyperplane H_l: switch from straight-line to next segment
    Position3d w_l; // center of hyperplane
    Position3d q_l; // normal vector to hyperplane

    // hyperplane H_ei: switch from second end spiral
    Position3d w_ei; // center of hyperplane
    Position3d q_ei; // normal vector to hyperplane

    // hyperplane H_e: end of Dubins path
    Position3d w_e; // center of hyperplane
    Position3d q_e; // normal vector to hyperplane

    Dubins3dPath() = default;

    double compute_length(const Waypoint3d& from, const Waypoint3d& to, double r_min, double gamma_max) {
        wp_s = from;
        wp_e = to;
        // We determine the configuration of the desired path: low altitude, medium altitude or high altitude
        // This needs, first, to calculate the length of the dubins 2d path
        DubinsPath path2d;
        double orig[3] = {from.x, from.y, from.dir};
        double dest[3] = {to.x, to.y, to.dir};

        dubins_init(orig, dest, r_min, &path2d);

        double L_2d = dubins_path_length(&path2d);

        double delta_z = to.z - from.z;
        if (abs(delta_z) <= L_2d*tan(gamma_max)) {
            /* Low altitude case: Just perform the 2d path with a linear increment in z */
            gamma = atan(delta_z/L_2d);
            configuration = Dubins3dPathConfiguration::SLS;
            R = r_min;
            L = L_2d / cos(gamma);
            k_s = 0; // zero spirals at the beginning
            k_e = 0; // zero spirals at the end
            return L;

        }
        if (abs(delta_z) >= (L_2d + 2 * M_PI * r_min) * tan(gamma_max)) {
            /* High altitude case */

            int k = static_cast<int>((abs(delta_z) / tan(gamma_max) - L_2d) / ( 2 * M_PI * r_min));
            if (delta_z>=0) {
                k_s = k, k_e = 0; // k spirals at the beginning
            } else {
                k_s = 0; k_e = k; // k spirals at the beginning
            }

            RadiusOptimizationResult optimal = get_optimal_climb_radius_and_length(
                    from, to, r_min, gamma_max, Dubins2dPathConfiguration::LSL, k, delta_z); //DubinsType unused
            R = optimal.R;
            L = (optimal.L + 2 * M_PI * k * R)/cos(gamma_max);
            configuration = Dubins3dPathConfiguration::SLS;
            gamma = delta_z >= 0 ? gamma_max : -gamma_max; // gamma > 0 means up.
            return L;

        }

        /* Medium altitude case */
        ASSERT(0 == 1);
        //FIXME: NOT IMPLEMENTED
        if (delta_z > 0) {
            gamma = gamma_max;
            configuration = Dubins3dPathConfiguration::SSLS;
            //FIXME: NOT IMPLEMENTED ADD_HELIX_AT_BEGINNING
        } else {
            gamma = -gamma_max;
            configuration = Dubins3dPathConfiguration::SLSS;
            //FIXME: NOT IMPLEMENTED ADD_HELIX_AT_END
        }
        R = r_min;
        L = L_2d/cos(gamma_max);
        return L;
    }

    /* See: Beard R.W., McLain T.W., Implementing Dubins Airplane Paths on Fixed-wing UAVs, 2013. */
    bool compute_path(Waypoint3d& from, Waypoint3d& to, double r_min, double gamma_max) {

        compute_length(from, to, r_min, gamma_max);

        return true;
    }

private:

    struct RadiusOptimizationResult {
        double R; // Optimal radius
        double L; // Length of the resulting trajectory
    };

    const double MAX_LOOPS = 100;
    const double R_TOLERANCE = 0.1;

    /* Find the optimal turning radius when climbing using the bisection method */
    RadiusOptimizationResult
    get_optimal_climb_radius_and_length(const Waypoint3d& start, const Waypoint3d& end, double r_min, double gamma_max,
                                        Dubins2dPathConfiguration /*unused*/ path2dconf, int k, double delta_z) {

        double R_l = r_min; // Lowest acceptable turn radius
        double R_h = 2 * r_min; // Highest acceptable turn radius
        double R = (R_l + R_h) / 2;

        DubinsPath path2d;
        double L_2d = 0.;

        double err = 1.;
        int n_loops = 0;
        while (abs(err) > R_TOLERANCE) {
            double orig[3] = {start.x, start.y, start.dir};
            double dest[3] = {end.x, end.y, end.dir};
            int ret = dubins_init(orig, dest, r_min, &path2d);
            ASSERT(ret == 0);
            double L_2d = dubins_path_length(&path2d); // We could speedup this function by using the DubinsType
            err = (L_2d + 2 * M_PI * k * R) * tan(gamma_max) - abs(delta_z);
            if (err > 0) {R_h = R;} else {R_l = R;}
            R = (R_l + R_h) / 2;

            ASSERT(++n_loops < MAX_LOOPS); // This loop should converge faster than MAX_LOOPS
        }

        return RadiusOptimizationResult{R, L_2d};

    }
};

#endif //PLANNING_CPP_DUBINS3D_H
