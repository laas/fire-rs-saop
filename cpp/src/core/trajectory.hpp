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

#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#include <algorithm>
#include <cassert>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <boost/log/trivial.hpp>

#include "../ext/dubins.h"
#include "../ext/json.hpp"
#include "../ext/optional.hpp"
#include "../utils.hpp"
#include "uav.hpp"
#include "waypoint.hpp"

namespace SAOP {

    using json = nlohmann::json;

    struct TrajectoryManeuver {
        Segment3d maneuver;
        double time;
        std::string name;

        TrajectoryManeuver(Segment3d seg, double time, std::string name)
                : maneuver(seg), time(time), name(name) {}

        TrajectoryManeuver(Waypoint3d wp, double time, std::string name)
                : maneuver(Segment3d(wp)), time(time), name(name) {}
    };

    static size_t UNIQUE_TRAJ_N = 0;
    [[maybe_unused]]
    static size_t UNIQUE_MAN_N = 0;

    struct TrajectoryConfig {
        std::string id_unique;
        UAV uav;
        double start_time;
        opt<Waypoint3d> start_position;
        opt<Waypoint3d> end_position;
        double max_flight_time;
        WindVector wind;

        TrajectoryConfig(const UAV& uav,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max(),
                         WindVector wind = WindVector(0., 0.))
                : uav(uav),
                  start_time(start_time),
                  start_position(opt<Waypoint3d>{}),
                  end_position(opt<Waypoint3d>{}),
                  max_flight_time(max_flight_time),
                  wind(wind) {
            id_unique = "traj" + std::to_string(UNIQUE_TRAJ_N);
            UNIQUE_TRAJ_N++;
        }

        TrajectoryConfig(const UAV& uav,
                         const Waypoint3d& start_position,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max(),
                         WindVector wind = WindVector(0., 0.))
                : uav(uav),
                  start_time(start_time),
                  start_position(start_position),
                  end_position(opt<Waypoint3d>()),
                  max_flight_time(max_flight_time),
                  wind(wind) {
            id_unique = "traj" + std::to_string(UNIQUE_TRAJ_N);
            UNIQUE_TRAJ_N++;
        }

        TrajectoryConfig(const UAV& uav,
                         const Waypoint3d& start_position,
                         const Waypoint3d& end_position,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max(),
                         WindVector wind = WindVector(0., 0.))
                : uav(uav),
                  start_time(start_time),
                  start_position(start_position),
                  end_position(end_position),
                  max_flight_time(max_flight_time),
                  wind(wind) {
            id_unique = "traj" + std::to_string(UNIQUE_TRAJ_N);
            UNIQUE_TRAJ_N++;
        }

        TrajectoryConfig(std::string name,
                         UAV uav,
                         Waypoint3d start_position,
                         Waypoint3d end_position,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max(),
                         WindVector wind = WindVector(0., 0.))
                : id_unique(std::move(name)),
                  uav(uav),
                  start_time(start_time),
                  start_position(std::move(start_position)),
                  end_position(std::move(end_position)),
                  max_flight_time(max_flight_time),
                  wind(wind) {}

        /*Used for python pickle support*/
        TrajectoryConfig(std::string name,
                         UAV uav,
                         opt<Waypoint3d> start_position,
                         opt<Waypoint3d> end_position,
                         double start_time,
                         double max_flight_time,
                         WindVector wind)
                : id_unique(std::move(name)),
                  uav(uav),
                  start_time(start_time),
                  start_position(start_position),
                  end_position(end_position),
                  max_flight_time(max_flight_time),
                  wind(wind) {}
    };

    class Trajectory {
    public:
        Trajectory(const Trajectory& trajectory) = default;

        /*Trajectory copy with modifiable segment interval*/
        Trajectory(const Trajectory& trajectory, IndexRange mod_range)
                : Trajectory(trajectory) {
            insertion_range = mod_range;
        };

        /* Empty trajectory constructor */
        explicit Trajectory(const TrajectoryConfig& config);

        Trajectory(TrajectoryConfig config, const std::vector<TrajectoryManeuver>& maneuvers);

        const TrajectoryConfig& conf() const {
            return config;
        }

        std::string name() const {
            return config.id_unique;
        }

        /* Number of segments in the trajectory. */
        size_t size() const { return _maneuvers.size(); }

        /* Time (s) at which the trajectory starts. */
        double start_time() const { return config.start_time; }

        /* Time (s) at which the trajectory ends. */
        double end_time() const {
            return size() == 0 ? start_time() : end_time(size() - 1);
        }

        /* Time (s) at which the UAV reaches the start waypoint of a given segment. */
        double start_time(size_t man_index) const {
            ASSERT(man_index < size());
            return _start_times[man_index];
        }

        /* Time (s) at which the UAV reaches the end waypoint of a given segment. */
        double end_time(size_t segment_index) const {
            ASSERT(segment_index < size());
            return start_time(segment_index) +
                   _maneuvers[segment_index].length / config.uav.max_air_speed();
        }

        /* Duration (s) of the path. */
        double duration() const {
            return end_time() - start_time();
        }

        /* Length (m) of the path. */
        double length() const {
            if (config.wind) {
                std::cerr << "Unprecise length";
            }
            return config.uav.max_air_speed() * duration();
        }

        /* Returns true if the first/last segments matches the ones given in the configuration
         * of the trajectory. */
        bool start_and_end_positions_respected() const {
            const bool start_valid = !config.start_position ||
                                     (size() >= 1 &&
                                      _maneuvers[0].start == *config.start_position &&
                                      _maneuvers[0].end == *config.start_position &&
                                      _maneuvers[0].length == 0);
            const size_t last_index = size() - 1;
            const bool end_valid = !config.end_position ||
                                   (size() >= 1 &&
                                    _maneuvers[last_index].start == *config.end_position &&
                                    _maneuvers[last_index].end == *config.end_position &&
                                    _maneuvers[last_index].length == 0);
            return start_valid && end_valid;
        }

        /* Returns true if this trajectory does not go over the maximum flight time given in configuration. */
        bool has_valid_flight_time() const {
            return ALMOST_LESSER_EQUAL(duration(), config.max_flight_time);
        }

        /* Index of the first modifiable segment. */
        size_t modifiable_size() const {
            return insertion_range.is_empty() ? 0 : insertion_range.end - insertion_range.start;
        }

        /* Index of the first modifiable segment. */
        size_t insertion_range_start() const {
            return insertion_range.start;
        }

        /* Index of the first modifiable segment. */
        size_t first_modifiable_maneuver() const {
            return insertion_range_start();
        }

        /* Index of the last modifiable segment */
        size_t insertion_range_end() const {
            return insertion_range.end;
        }

        /* Index of the last modifiable segment */
        size_t last_modifiable_maneuver() const {
            return insertion_range_end() > 0 ? insertion_range_end() - 1 : 0;
        }

        /* Random segment index selected among modifiable segments */
        opt<size_t> random_modifiable_id() const {
            if (insertion_range_start() >= insertion_range_end()) {
                return {};
            }
            return rand(insertion_range_start(), insertion_range_end());
        }

        /* Random segment index selected among modifiable segments */
        opt<size_t> random_insertion_id() const {
            if (insertion_range_start() > insertion_range_end()) {
                return {};
            }
            return rand(insertion_range_start(), insertion_range_end() + 1);
        }

        /* Accesses the index-th segment of the trajectory */
        TrajectoryManeuver operator[](size_t index) const {
            return TrajectoryManeuver{_maneuvers[index], _start_times[index], _man_names[index]};
        }

        /* Accesses the index-th segment of the trajectory */
        Segment3d segment(size_t index) const { return _maneuvers[index]; }

        // PyBind11 won't cast our opt<T> as std::experimental::optional so these functions cannot be used in python
        // FIXME: Switch to C++14 and use std::experimental::optional
        opt<Segment3d> base_start() const { return config.start_position ? _maneuvers.front() : opt<Segment3d>{}; }

        opt<Segment3d> base_end() const { return config.end_position ? _maneuvers.back() : opt<Segment3d>{}; }

        const std::vector<std::string>& names() const { return _man_names; }

        /* Only for python interface */
        const std::vector<Segment3d>& segments() const { return _maneuvers; };

        std::vector<Segment3d>::const_iterator segments_begin() const { return _maneuvers.begin(); };

        std::vector<Segment3d>::const_iterator segments_end() const { return _maneuvers.end(); };

        /* Only for python interface */
        const std::vector<double>& start_times() const { return _start_times; };

        std::vector<double>::const_iterator start_times_begin() const { return _start_times.begin(); };

        std::vector<double>::const_iterator start_times_end() const { return _start_times.end(); };

        /* Only for python interface */
        std::vector<std::string>::const_iterator names_begin() const { return _man_names.begin(); };

        std::vector<std::string>::const_iterator names_end() const { return _man_names.end(); };


//    /* Returns the part of a Trajectory within the IndexRange as a new Trajectory.
//     * For each cut argument, if it is true: fixed start/end on cut
//     * false: free start/end on cut
//     * unspecified: keep original start/end */
//    Trajectory
//    slice(IndexRange range, opt<bool> cut_start_time={}, opt<bool> cut_max_flight_time={}) const {
//        if (size() == 0) {
//            // If the trajectory is void, a subtrajectory is the trajectory
//            std::cerr << "Slicing an empty trajectory" << std::endl;
//            return Trajectory(*this);
//        }
//
//        auto traj_range = IndexRange(0, size()).intersection_with(range);
//
//        if (!traj_range.is_empty()) {
//             return Trajectory(TrajectoryConfig(conf.uav, start_time(traj_range.start),
//                                               end_time(traj_range.end - 1)));
//        }
//
//        if (!cut_start_time) {
//            cut_start_time = conf.start_position ? true : false;
//        } else if (!cut_max_flight_time) {
//            cut_max_flight_time = conf.end_position ? true : false;
//        }
//
//        if (conf.start_position && conf.end_position) {
//            TrajectoryConfig new_conf = TrajectoryConfig(
//                    conf.uav,
//                    *conf.start_position,
//                    *conf.end_position,
//                    cut_start_time ? start_time(traj_range.start) : conf.start_time ,
//                    cut_max_flight_time ? end_time(traj_range.end - 1) : conf.max_flight_time);
//        } else if (conf.start_position ) {
//
//        } else {
//            TrajectoryConfig new_conf = TrajectoryConfig(
//                    conf.uav,
//                    cut_start_time ? start_time(traj_range.start) : conf.start_time ,
//                    cut_max_flight_time ? end_time(traj_range.end - 1) : conf.max_flight_time);
//        }
//
//
//        Trajectory new_trajectory = Trajectory(new_conf);
//        for (size_t i=*new_traj_start_i, j= new_trajectory.first_modifiable_id(); i<=new_traj_end_i; ++i, ++j) {
//            new_trajectory.insert_segment(traj[i], j);
//        }
//
//
//    }

        /* Make all the maneuvers before 'time' unmodifiable. 'time' is still modifiable'*/
        void freeze_before(double time);

        /* Make all the maneuvers before 'man_index' unmodifiable. 'man_index' is still modifiable'*/
        void freeze_before(size_t man_index);

        void freeze();

        bool can_modify(size_t man_index) const;

        bool can_insert_at(size_t man_index) const;

        std::vector<bool> modifiable() const;

        /* Returns the part of a Trajectory within the TimewWindow as a new Trajectory. */
        Trajectory slice(TimeWindow tw) const;

        Trajectory with_longer_segments(double inc_length);

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::vector<Waypoint3d> as_waypoints() const;

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>> as_waypoints_with_time() const;

        /* Returns the trajectory as a set of sequences of waypoints, time and maneuver names.*/
        std::tuple<std::vector<Waypoint3d>, std::vector<double>, std::vector<std::string>> as_waypoints_time_name() const;

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::vector<Waypoint3d> sampled(const double step_size = 1) const;

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>> sampled_with_time(double step_size = 1) const;

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::pair<std::vector<Waypoint3d>, std::vector<double>>
        sampled_with_time(TimeWindow time_range, double step_size = 1) const;

        double insertion_duration_cost(size_t insert_loc, const Segment3d segment) const;

        double removal_duration_gain(size_t index) const;

        /* Increase in time (s) as a result of replacing the segment at the given index by the one provided.*/
        double replacement_duration_cost(size_t index, const Segment3d& segment) const {
            return replacement_duration_cost(index, std::vector<Segment3d>{segment});
        }

        /* Increase in time (s) as a result of replacing the N segments at the given index by the N segemnts provided.*/
        double replacement_duration_cost(size_t index, const std::vector<Segment3d>& segments) const {
            return replacement_duration_cost(index, segments.size(), segments);
        }

        /* Increase in time (s) as a result of replacing n segments at the given index by the N segments provided.*/
        double
        replacement_duration_cost(size_t index, size_t n_replaced, const std::vector<Segment3d>& segments) const;

        /* Returns a new trajectory with the given waypoint appended (as a segment of length 0) */
        Trajectory with_waypoint_at_end(Waypoint3d& waypoint) const {
            Segment3d s(waypoint);
            return with_segment_at_end(s);
        }

        /* Returns a new Trajectory with the given segment appended. */
        Trajectory with_segment_at_end(const Segment3d& segment) const {
            return with_additional_segment(size(), segment);
        }

        /* Returns a new trajectory with an additional segment at the given index */
        Trajectory with_additional_segment(size_t index, const Segment3d& seg) const;

        /* In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
        Trajectory with_replaced_section(size_t index, const std::vector<Segment3d>& segments) const;

        /* Adds segment to the end of the trajectory */
        void append_segment(const Segment3d& seg);

        /* Inserts the given segment at the given index */
        void insert_segment(const Segment3d& seg, size_t at_index);

        /* Returns a new trajectory without the segment at the given index */
        Trajectory without_segment(size_t index) const;

        /* Removes the segment at the given index. */
        void erase_all_modifiable_maneuvers();

        /* Removes the segment at the given index. */
        void erase_segment(size_t at_index);

        void replace_segment(size_t at_index, const Segment3d& by_segment);

        void replace_section(size_t index, const std::vector<Segment3d>& segments);

        std::string to_string() const;

    private:
        TrajectoryConfig config;

        std::vector<Segment3d> _maneuvers;
        std::vector<double> _start_times;
        std::vector<std::string> _man_names;

        /* Boolean flag that is set to true once the trajectory is initialized.
         * Validity checks are only performed when this flag is true. */
        bool is_set_up = false;

        /* Range of vector<Segment3d> ids where segments can be modified, removed or inserted.
         * This is independent of the start and end points that can be fixed independently of this
         * setting */
        IndexRange insertion_range = IndexRange::unbounded(); // All by default

        /* Functions that asserts invariants of a trajectory.
         * This is for debugging purpose only and the function content should be commented out. */
        void check_validity() const;

        /* Converts a vector of waypoints to a vector of segments. */
        static std::vector<Segment3d> segments_from_waypoints(std::vector<Waypoint3d> waypoints);

        /* Computes the cost of a sub-trajectory composed of the given segments. */
        double segments_duration(const std::vector<Segment3d>& segments, size_t start, size_t length) const;

        /* Adds segment to the end of the trajectory with custom name*/
        void append_segment(const Segment3d& seg, std::string name);

        /* Inserts the given segment at the given index with a custom name */
        void insert_segment(const Segment3d& seg, size_t at_index, std::string name);
    };

    [[maybe_unused]]
    static void to_json(json& j, const Trajectory& traj) {
        j = json{{"name",           traj.name()},
                 {"max_duration",   traj.conf().max_flight_time},
                 {"maneuver_names", traj.names()},
                 {"duration",       traj.duration()},
                 {"num_segments",   traj.size()},
                 {"start_time",     traj.start_time()},
                 {"start_times",    traj.start_times()},
                 {"end_time",       traj.end_time()},
                 {"segments",       traj.segments()}};
    }

}
#endif //PLANNING_CPP_TRAJECTORY_H
