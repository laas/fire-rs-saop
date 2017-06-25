#ifndef PLANNING_CPP_UAV_H
#define PLANNING_CPP_UAV_H

#include <vector>
#include <cassert>
#include "ext/dubins.h"
#include "waypoint.h"
#include "utils.h"
#include "trajectory.h"


struct UAV {
    double max_angular_velocity;
    double max_air_speed;
    double min_turn_radius;
    double view_width = 100;
    double view_depth = 70;


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

    /** Rotates the given segment on the center of the visibility area. */
    Segment rotate_on_visibility_center(const Segment& segment, double target_dir) const {
        const double visibility_depth = segment.length + view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth/2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth/2;
        const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth/2;
        const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth/2;

        return Segment(Waypoint(new_segment_start_x, new_segment_start_y, target_dir), segment.length);
    }

    /** Builds a new segment of the given length and direction,
     *  such that (x_coords, y_coords) is at the center of the visibility area. */
    Segment observation_segment(double x_coords, double y_coords, double dir, double length) const {
        const double visibility_depth = length + view_depth;
        const double segment_start_x = x_coords - cos(dir) * visibility_depth/2;
        const double segment_start_y = y_coords - sin(dir) * visibility_depth/2;
        return Segment(Waypoint(segment_start_x, segment_start_y, dir), length);
    }

    Waypoint visibilty_center(const Segment& segment) const {
        const double visibility_depth = segment.length + view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth/2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth/2;
        return Waypoint(vis_center_x, vis_center_y, segment.start.dir);
    }



private:
    DubinsPath dubins_path(const Waypoint &origin, const Waypoint &target) const {
        DubinsPath path;
        double orig[3] = { origin.x, origin.y, origin.dir };
        double dest[3] = { target.x, target.y, target.dir };
        // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
        int ret = dubins_init(orig, dest, min_turn_radius, &path);
        ASSERT(ret == 0);
        return path;
    }
};



#endif //PLANNING_CPP_UAV_H
