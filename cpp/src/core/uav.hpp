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

#ifndef PLANNING_CPP_UAV_H
#define PLANNING_CPP_UAV_H

#include <cassert>
#include <vector>

#include "../ext/dubins.h"
#include "../utils.hpp"
#include "dubins3d.hpp"
#include "dubinswind.hpp"
#include "waypoint.hpp"

namespace SAOP {

    struct UAV {

        UAV(const UAV& uav) = default;

        UAV(std::string name, const double max_air_speed, const double max_angular_velocity, const double max_pitch_angle) :
                name_unique(std::move(name)),
                _max_angular_velocity(max_angular_velocity),
                _max_air_speed(max_air_speed),
                _min_turn_radius(max_air_speed / max_angular_velocity),
                _max_pitch_angle(max_pitch_angle) {}

        std::string name() const {
            return name_unique;
        }

        double max_angular_velocity() const {
            return _max_angular_velocity;
        }

        double max_air_speed() const {
            return _max_air_speed;
        }

        double min_turn_radius() const {
            return _min_turn_radius;
        }

        double max_pitch_angle() const {
            return _max_pitch_angle;
        }

        double view_width() const {
            return _view_width;
        }

        double view_depth() const {
            return _view_depth;
        }

        /* Determine whether the UAV is turning*/
        bool is_turning(const Waypoint3d& prev, const    Waypoint3d& current, double epsilon=0.09) const {
            // r = dist(prev, current) / (current.dir - prev.dir)
            // roll = atan(v^2/(r*g))
            return !ALMOST_EQUAL_EPS(prev.dir, current.dir, epsilon);
        }

        /** Returns the Dubins travel distance between the two waypoints. */
        double travel_distance(const Waypoint& origin, const Waypoint& target) const;

        /** Returns the Dubins travel distance between the two waypoints. */
        double travel_distance(const Waypoint3d& origin, const Waypoint3d& target) const;

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Waypoint& origin, const Waypoint& target) const;

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Waypoint3d& origin, const Waypoint3d& target) const;

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind) const;

        /** Returns the travel time between the two waypoints. */
        double travel_time(const Segment3d& segment, const WindVector& wind) const;

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint>
        path_sampling(const Waypoint& origin, const Waypoint& target, double step_size) const;

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint3d>
        path_sampling(const Waypoint3d& origin, const Waypoint3d& target, double step_size) const;

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint3d>
        path_sampling(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                      double step_size) const;

        /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
        std::vector<Waypoint3d>
        path_sampling_airframe(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                               double step_size) const;

        std::vector<Waypoint3d> segment_sampling(const Segment3d& segment, const WindVector& wind, double step_size);

        std::pair<std::vector<Waypoint3d>, std::vector<double>>
        segment_sampling_with_time(const Segment3d& segment, const WindVector& wind, double step_size, double t_start);

        /* Returns a sequence of waypoints with its corresponding time following the dubins trajectory,
         * one every step_size distance units. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>>
        path_sampling_with_time(const Waypoint3d& origin, const Waypoint3d& target, double step_size,
                                double t_start) const;

        /* Returns a sequence of waypoints with its corresponding time following the dubins trajectory,
         * one every step_size distance units. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>>
        path_sampling_with_time(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                                double step_size, double t_start) const;

        /** Rotates the given segment on the center of the visibility area. */
        Segment rotate_on_visibility_center(const Segment& segment, double target_dir) const;

        /** Rotates the given segment on the center of the visibility area. */
        Segment3d rotate_on_visibility_center(const Segment3d& segment, double target_dir) const;

        /** Builds a new segment of the given length and direction,
         *  such that (x_coords, y_coords) is at the center of the visibility area. */
        Segment observation_segment(double x_coords, double y_coords, double dir, double length) const;

        /** Builds a new segment of the given length and direction,
         *  such that (x_coords, y_coords) is at the center of the visibility area. */
        Segment3d
        observation_segment(double x_coords, double y_coords, double z_coords, double dir, double length) const;

        Waypoint visibility_center(const Segment& segment) const;

        Waypoint3d visibility_center(const Segment3d& segment) const;

    private:
        std::string name_unique;
        double _max_angular_velocity;
        double _max_air_speed;
        double _min_turn_radius;
        double _max_pitch_angle; /* aka gamma_max */
        double _view_width = 50;
        double _view_depth = 50;

        DubinsPath dubins_path(const Waypoint& origin, const Waypoint& target) const {
            DubinsPath path;
            double orig[3] = {origin.x, origin.y, origin.dir};
            double dest[3] = {target.x, target.y, target.dir};
            // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
            int ret = dubins_init(orig, dest, _min_turn_radius, &path);
            ASSERT(ret == 0);
            return path;
        }
    };
}
#endif //PLANNING_CPP_UAV_H
