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

#include "uav.hpp"

namespace SAOP {

    /** Returns the Dubins travel distance between the two waypoints. */
    double UAV::travel_distance(const Waypoint& origin, const Waypoint& target) const {
        DubinsPath path = dubins_path(origin, target);
        return dubins_path_length(&path);
    }

    /** Returns the Dubins travel distance between the two waypoints. */
    double UAV::travel_distance(const Waypoint3d& origin, const Waypoint3d& target) const {
        Dubins3dPathLength path(origin, target, _min_turn_radius, _max_pitch_angle);
        return path.L;
    }

    /** Returns the travel time between the two waypoints. */
    double UAV::travel_time(const Waypoint& origin, const Waypoint& target) const {
        return travel_distance(origin, target) / _max_air_speed;
    }

    /** Returns the travel time between the two waypoints. */
    double UAV::travel_time(const Waypoint3d& origin, const Waypoint3d& target) const {
        return travel_distance(origin, target) / _max_air_speed;
    }

    /** Returns the travel time between the two waypoints. */
    double UAV::travel_time(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind) const {
        try {
            DubinsWind path(origin, target, wind, _max_air_speed, _min_turn_radius);
            return path.T();
        } catch (const DubinsWindPathNotFoundException& e) {
#ifdef DEBUG
            std::cerr << e.what() << std::endl;
#endif
            return std::numeric_limits<double>::infinity();
        }
    }

    /** Returns the travel time between the two waypoints. */
    double UAV::travel_time(const Segment3d& segment, const WindVector& wind) const {
        auto v = WindVector(_max_air_speed * cos(segment.start.dir),
                            _max_air_speed * sin(segment.start.dir));
        auto vw = (v + wind);
        auto v_eff = vw.modulo() * cos(-vw.dir() + v.dir());
        ASSERT(v_eff > 0);
        return segment.length / v_eff;
    }

    /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
    std::vector<Waypoint>
    UAV::path_sampling(const Waypoint& origin, const Waypoint& target, double step_size) const {
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
    UAV::path_sampling(const Waypoint3d& origin, const Waypoint3d& target, double step_size) const {
        ASSERT(step_size > 0);
        Dubins3dPath path = Dubins3dPath(origin, target, _min_turn_radius, _max_pitch_angle);
        std::vector<Waypoint3d> waypoints;
        for (double it = 0; it < path.L_2d; it += step_size) {
            waypoints.push_back(path.sample(it));
        }
        waypoints.push_back(target);
        return waypoints;
    }

    /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
    std::vector<Waypoint3d>
    UAV::path_sampling(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                       double step_size) const {
        ASSERT(step_size > 0);
        DubinsWind path = DubinsWind(origin, target, wind, _max_air_speed, _min_turn_radius);
        return path.sampled(step_size);
    }

    /** Returns a sequence of waypoints following the dubins trajectory, one every step_size distance units. */
    std::vector<Waypoint3d>
    UAV::path_sampling_airframe(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                                double step_size) const {
        ASSERT(step_size > 0);
        DubinsWind path = DubinsWind(origin, target, wind, _max_air_speed, _min_turn_radius);
        return path.sampled_airframe(step_size);
    }

    std::vector<Waypoint3d> UAV::segment_sampling(const Segment3d& segment, const WindVector& wind, double step_size) {
        ASSERT(step_size > 0);
        std::vector<Waypoint3d> waypoints;

        for (double i = 0; i < segment.length; i += step_size) {
            waypoints.push_back(segment.start.forward(i));
        }
        waypoints.push_back(segment.end);

        return waypoints;
    }

    std::pair<std::vector<Waypoint3d>, std::vector<double>>
    UAV::segment_sampling_with_time(const Segment3d& segment, const WindVector& wind, double step_size,
                                    double t_start) {
        ASSERT(step_size > 0);
        auto v = WindVector(_max_air_speed * cos(segment.start.dir),
                            _max_air_speed * sin(segment.start.dir));
        auto vw = (v + wind);
        auto v_eff = vw.modulo() * cos(-vw.dir() + v.dir());

        std::vector<Waypoint3d> waypoints;
        std::vector<double> times;

        for (double i = 0; i < segment.length; i += step_size) {
            waypoints.push_back(segment.start.forward(i));
            times.push_back(t_start + i / v_eff);
        }
        waypoints.push_back(segment.end);
        times.push_back(t_start + segment.length / v_eff);

        return {waypoints, times};
    }

    /* Returns a sequence of waypoints with its corresponding time following the dubins trajectory,
     * one every step_size distance units. */
    std::pair<std::vector<Waypoint3d>, std::vector<double>>
    UAV::path_sampling_with_time(const Waypoint3d& origin, const Waypoint3d& target, double step_size,
                                 double t_start) const {
        ASSERT(step_size > 0);
        Dubins3dPath path = Dubins3dPath(origin, target, _min_turn_radius, _max_pitch_angle);
        std::vector<Waypoint3d> waypoints;
        std::vector<double> times;

        for (double it = 0; it < path.L_2d; it += step_size) {
            waypoints.push_back(path.sample(it));
            times.push_back(t_start + it / _max_air_speed);
        }
        waypoints.push_back(target);
        times.push_back(t_start + path.L_2d / _max_air_speed);
        return {waypoints, times};
    }

    /* Returns a sequence of waypoints with its corresponding time following the dubins trajectory,
     * one every step_size distance units. */
    std::pair<std::vector<Waypoint3d>, std::vector<double>>
    UAV::path_sampling_with_time(const Waypoint3d& origin, const Waypoint3d& target, const WindVector& wind,
                                 double step_size, double t_start) const {
        ASSERT(step_size > 0);
        DubinsWind path = DubinsWind(origin, target, wind, _max_air_speed, _min_turn_radius);
        auto wp_time = path.sampled_with_time(step_size);
        auto& t = std::get<1>(wp_time);
        for (auto it = t.begin(); it != t.end(); ++it) {
            *it += t_start;
        }
        return wp_time;

    }

    /** Rotates the given segment on the center of the visibility area. */
    Segment UAV::rotate_on_visibility_center(const Segment& segment, double target_dir) const {
        const double visibility_depth = segment.length + _view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
        const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth / 2;
        const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth / 2;

        return Segment(Waypoint(new_segment_start_x, new_segment_start_y, target_dir), segment.length);
    }

    /** Rotates the given segment on the center of the visibility area. */
    Segment3d UAV::rotate_on_visibility_center(const Segment3d& segment, double target_dir) const {
        ASSERT(ALMOST_EQUAL(segment.start.z, segment.end.z));
        const double visibility_depth = segment.length + _view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
        const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth / 2;
        const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth / 2;
        return Segment3d(Waypoint3d(new_segment_start_x, new_segment_start_y, segment.start.z, target_dir),
                         segment.length);
    }

    /** Builds a new segment of the given length and direction,
     *  such that (x_coords, y_coords) is at the center of the visibility area. */
    Segment UAV::observation_segment(double x_coords, double y_coords, double dir, double length) const {
        const double visibility_depth = length + _view_depth;
        const double segment_start_x = x_coords - cos(dir) * visibility_depth / 2;
        const double segment_start_y = y_coords - sin(dir) * visibility_depth / 2;
        return Segment(Waypoint(segment_start_x, segment_start_y, dir), length);
    }

    /** Builds a new segment of the given length and direction,
     *  such that (x_coords, y_coords) is at the center of the visibility area. */
    Segment3d
    UAV::observation_segment(double x_coords, double y_coords, double z_coords, double dir, double length) const {
        const double visibility_depth = length + _view_depth;
        const double segment_start_x = x_coords - cos(dir) * visibility_depth / 2;
        const double segment_start_y = y_coords - sin(dir) * visibility_depth / 2;
        return Segment3d(Waypoint3d(segment_start_x, segment_start_y, z_coords, dir), length);
    }

    Waypoint UAV::visibility_center(const Segment& segment) const {
        const double visibility_depth = segment.length + _view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
        return Waypoint(vis_center_x, vis_center_y, segment.start.dir);
    }

    Waypoint3d UAV::visibility_center(const Segment3d& segment) const {
        ASSERT(ALMOST_EQUAL(segment.start.z, segment.end.z));
        const double visibility_depth = segment.length + _view_depth;
        const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth / 2;
        const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth / 2;
        return Waypoint3d(vis_center_x, vis_center_y, segment.start.z, segment.start.dir);
    }
}