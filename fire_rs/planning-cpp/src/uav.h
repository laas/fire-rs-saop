#ifndef PLANNING_CPP_UAV_H
#define PLANNING_CPP_UAV_H

#include <vector>
#include <cassert>
#include "dubins.h"
#include "waypoint.h"


struct UAV {
    double max_angular_velocity;
    double max_air_speed;
    double min_turn_radius;
    double view_width = 100;
    double view_depth = 100;


     UAV(const double max_air_speed, const double max_angular_velocity) :
            max_angular_velocity(max_angular_velocity),
            max_air_speed(max_air_speed),
            min_turn_radius(max_air_speed / max_angular_velocity) {
        printf("%f -- %f\n", max_air_speed, min_turn_radius);
    };

    UAV(const UAV& uav) = default;

    /** Returns the Dubins travel distance between the two waypoints. */
    double travel_distance(const Waypoint &origin, const Waypoint &target) const {
        DubinsPath path = dubins_path(origin, target);
        return dubins_path_length(&path);
    }

    /** Returns the travel time between the two waypoints. */
    double travel_time(const Waypoint& origin, const Waypoint &target) const {
        return travel_distance(origin, target) / max_air_speed;
    }

    /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
    std::vector<Waypoint> path_sampling(const Waypoint &origin, const Waypoint &target, const double step_size) const {
        assert(step_size > 0);
        const double length = travel_distance(origin, target);
        DubinsPath path = dubins_path(origin, target);
        std::vector<Waypoint> waypoints;
        for(double it=0; it<length; it += step_size) {
            double q[3];
            dubins_path_sample(&path, it, q);
            Waypoint wp(q[0], q[1], q[2]);
            waypoints.push_back(wp);
        }
        waypoints.push_back(target);
        return waypoints;
    }

private:
    DubinsPath dubins_path(const Waypoint &origin, const Waypoint &target) const {
        DubinsPath path;
        // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
        int ret = dubins_init((double*) &origin, (double*) &target, min_turn_radius, &path);
        assert(ret == 0);
        return path;
    }
};



#endif //PLANNING_CPP_UAV_H
