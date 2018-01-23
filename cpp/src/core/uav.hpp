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

#ifndef PLANNING_CPP_UAV_H
#define PLANNING_CPP_UAV_H

#include <vector>
#include <cassert>
#include "../ext/dubins.h"
#include "waypoint.hpp"
#include "../utils.hpp"
#include "dubins3d.hpp"

namespace SAOP {

    struct UAV {
        const double max_angular_velocity;
        const double max_air_speed;
        const double min_turn_radius;
        const double max_pitch_angle; /* aka gamma_max */
        const double view_width = 100;
        const double view_depth = 70;


        UAV(const double max_air_speed, const double max_angular_velocity, const double max_pitch_angle) :
                max_angular_velocity(max_angular_velocity),
                max_air_speed(max_air_speed),
                min_turn_radius(max_air_speed / max_angular_velocity),
                max_pitch_angle(max_pitch_angle) {}

        UAV(const UAV& uav) = default;

        /** Returns the Dubins travel distance between the two waypoints. */
        double travel_distance(const Waypoint& origin, const Waypoint& target) const {
            DubinsPath path = dubins_path(origin, target);
            return dubins_path_length(&path);
        }

        /** Returns the Dubins travel distance between the two waypoints. */
        double travel_distance(const Waypoint3d& origin, const Waypoint3d& target) const {
            Dubins3dPathLength path(origin, target, min_turn_radius, max_pitch_angle);
            return path.L;
        }

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Waypoint& origin, const Waypoint& target) const {
            return travel_distance(origin, target) / max_air_speed;
        }

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Waypoint3d& origin, const Waypoint3d& target) const {
            return travel_distance(origin, target) / max_air_speed;
        }

        /* Determine whether the UAV is turning*/
        bool is_turning(const Waypoint3d& prev, const Waypoint3d& current) const {
            // r = dist(prev, current) / (current.dir - prev.dir)
            // roll = atan(v^2/(r*g))
            return !ALMOST_EQUAL(prev.dir, current.dir);
        }

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint>
        path_sampling(const Waypoint& origin, const Waypoint& target, const double step_size) const {
            ASSERT(step_size > 0);
            const double length = travel_distance(origin, target);
            DubinsPath path = dubins_path(origin, target);
            std::vector<Waypoint> waypoints;
            for (double it = 0; it < length; it += step_size) {
                double q[3];
                dubins_path_sample(&path, it, q);
                Waypoint wp(q[0], q[1], q[2]);
                waypoints.push_back(wp);
            }
            waypoints.push_back(target);
            return waypoints;
        }

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint3d>
        path_sampling(const Waypoint3d& origin, const Waypoint3d& target, const double step_size) const {
            ASSERT(step_size > 0);
            Dubins3dPath path = Dubins3dPath(origin, target, min_turn_radius, max_pitch_angle);
            std::vector<Waypoint3d> waypoints;
            for (double it = 0; it < path.L_2d; it += step_size) {
                waypoints.push_back(path.sample(it));
            }
            waypoints.push_back(target);
            return waypoints;
        }

        /* Returns a sequence of waypoints with its corresponding time following the dubins trajectory,
         * one every step_size distance units. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>>
        path_sampling_with_time(const Waypoint3d& origin, const Waypoint3d& target, const double step_size,
                                double t_start) const {
            ASSERT(step_size > 0);
            Dubins3dPath path = Dubins3dPath(origin, target, min_turn_radius, max_pitch_angle);
            std::vector<Waypoint3d> waypoints;
            std::vector<double> times;

            for (double it = 0; it < path.L_2d; it += step_size) {
                waypoints.push_back(path.sample(it));
                times.push_back(t_start + it / max_air_speed);
            }
            waypoints.push_back(target);
            times.push_back(t_start + path.L_2d / max_air_speed);
            return {waypoints, times};
        }

        /** Rotates the given segment on the center of the visibility area. */
        Segment rotate_on_visibility_center(const Segment& segment, double target_dir) const {
            const double visibility_depth = segment.length + view_depth;
            const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
            const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
            const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth / 2;
            const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth / 2;

            return Segment(Waypoint(new_segment_start_x, new_segment_start_y, target_dir), segment.length);
        }

        /** Rotates the given segment on the center of the visibility area. */
        Segment3d rotate_on_visibility_center(const Segment3d& segment, double target_dir) const {
            ASSERT(ALMOST_EQUAL(segment.start.z, segment.end.z));
            const double visibility_depth = segment.length + view_depth;
            const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
            const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
            const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth / 2;
            const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth / 2;
            return Segment3d(Waypoint3d(new_segment_start_x, new_segment_start_y, segment.start.z, target_dir),
                             segment.length);
        }

        /** Builds a new segment of the given length and direction,
         *  such that (x_coords, y_coords) is at the center of the visibility area. */
        Segment observation_segment(double x_coords, double y_coords, double dir, double length) const {
            const double visibility_depth = length + view_depth;
            const double segment_start_x = x_coords - cos(dir) * visibility_depth / 2;
            const double segment_start_y = y_coords - sin(dir) * visibility_depth / 2;
            return Segment(Waypoint(segment_start_x, segment_start_y, dir), length);
        }

        /** Builds a new segment of the given length and direction,
         *  such that (x_coords, y_coords) is at the center of the visibility area. */
        Segment3d
        observation_segment(double x_coords, double y_coords, double z_coords, double dir, double length) const {
            const double visibility_depth = length + view_depth;
            const double segment_start_x = x_coords - cos(dir) * visibility_depth / 2;
            const double segment_start_y = y_coords - sin(dir) * visibility_depth / 2;
            return Segment3d(Waypoint3d(segment_start_x, segment_start_y, z_coords, dir), length);
        }

        Waypoint visibility_center(const Segment& segment) const {
            const double visibility_depth = segment.length + view_depth;
            const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
            const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
            return Waypoint(vis_center_x, vis_center_y, segment.start.dir);
        }

        Waypoint3d visibility_center(const Segment3d& segment) const {
            ASSERT(ALMOST_EQUAL(segment.start.z, segment.end.z));
            const double visibility_depth = segment.length + view_depth;
            const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
            const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
            return Waypoint3d(vis_center_x, vis_center_y, segment.start.z, segment.start.dir);
        }

    private:
        DubinsPath dubins_path(const Waypoint& origin, const Waypoint& target) const {
            DubinsPath path;
            double orig[3] = {origin.x, origin.y, origin.dir};
            double dest[3] = {target.x, target.y, target.dir};
            // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
            int ret = dubins_init(orig, dest, min_turn_radius, &path);
            ASSERT(ret == 0);
            return path;
        }
    };
}
#endif //PLANNING_CPP_UAV_H
