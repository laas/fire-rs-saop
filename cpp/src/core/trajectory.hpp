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

#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <memory>
#include "../ext/dubins.h"
#include "waypoint.hpp"
#include "uav.hpp"
#include "../utils.hpp"
#include "../ext/optional.hpp"

namespace SAOP {

    struct TrajectoryConfig {
        UAV uav;
        double start_time;
        opt<Waypoint3d> start_position;
        opt<Waypoint3d> end_position;
        double max_flight_time;

        TrajectoryConfig(const UAV& uav,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max())
                : uav(uav),
                  start_time(start_time),
                  start_position((opt<Waypoint3d>) {}),
                  end_position((opt<Waypoint3d>) {}),
                  max_flight_time(max_flight_time) {}

        TrajectoryConfig(const UAV& uav,
                         const Waypoint3d& start_position,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max())
                : uav(uav),
                  start_time(start_time),
                  start_position(start_position),
                  end_position(opt<Waypoint3d>()),
                  max_flight_time(std::numeric_limits<double>::max()) {}

        TrajectoryConfig(const UAV& uav,
                         const Waypoint3d& start_position,
                         const Waypoint3d& end_position,
                         double start_time = 0,
                         double max_flight_time = std::numeric_limits<double>::max())
                : uav(uav),
                  start_time(start_time),
                  start_position(start_position),
                  end_position(end_position),
                  max_flight_time(max_flight_time) {}
    };

    struct TimedManeuver {
        Segment3d maneuver;
        double start_time;
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
        explicit Trajectory(const TrajectoryConfig& _config)
                : _conf(_config) {
            if (_conf.start_position) {
                append_segment(Segment3d(*_conf.start_position));
                insertion_range = IndexRange::end_unbounded(1);
            }

            if (_conf.end_position) {
                append_segment(Segment3d(*_conf.end_position));
                insertion_range = insertion_range.intersection_with(IndexRange::start_unbounded(size() - 1));
            }
            is_set_up = true;
            check_validity();
        }

        const TrajectoryConfig& conf() const {
            return _conf;
        }

        /* Number of segments in the trajectory. */
        size_t size() const { return _maneuvers.size(); }

        /* Time (s) at which the trajectory starts. */
        double start_time() const { return _conf.start_time; }

        /* Time (s) at which the trajectory ends. */
        double end_time() const {
            return size() == 0 ? start_time() : end_time(size() - 1);
        }

        /* Time (s) at which the UAV reaches the start waypoint of a given segment. */
        double start_time(size_t man_index) const {
            ASSERT(man_index >= 0 && man_index < size())
            return _start_times[man_index];
        }

        /* Time (s) at which the UAV reaches the end waypoint of a given segment. */
        double end_time(size_t segment_index) const {
            ASSERT(segment_index >= 0 && segment_index < size())
            return start_time(segment_index) +
                   _maneuvers[segment_index].length / _conf.uav.max_air_speed;
        }

        /* Duration (s) of the path. */
        double duration() const {
            return end_time() - start_time();
        }

        /* Length (m) of the path. */
        double length() const {
            return _conf.uav.max_air_speed * duration();
        }

        /* Returns true if the first/last segments matches the ones given in the configuration
         * of the trajectory. */
        bool start_and_end_positions_respected() const {
            const bool start_valid = !_conf.start_position ||
                                     (size() >= 1 &&
                                      _maneuvers[0].start == *_conf.start_position &&
                                      _maneuvers[0].end == *_conf.start_position &&
                                      _maneuvers[0].length == 0);
            const size_t last_index = size() - 1;
            const bool end_valid = !_conf.end_position ||
                                   (size() >= 1 &&
                                    _maneuvers[last_index].start == *_conf.end_position &&
                                    _maneuvers[last_index].end == *_conf.end_position &&
                                    _maneuvers[last_index].length == 0);
            return start_valid && end_valid;
        }

        /* Returns true if this trajectory does not go over the maximum flight time given in configuration. */
        bool has_valid_flight_time() const {
            return ALMOST_LESSER_EQUAL(duration(), _conf.max_flight_time);
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

//        /* Accesses the index-th segment of the trajectory */
        TimedManeuver operator[](size_t index) const { return TimedManeuver{_maneuvers[index], _start_times[index]}; }

        /* Accesses the index-th segment of the trajectory */
        Segment3d maneuver(size_t index) const { return _maneuvers[index]; }

        /* Only for python interface */
        std::vector<Segment3d> maneuvers() const { return _maneuvers; };

        std::vector<Segment3d>::const_iterator maneuvers_begin() const { return _maneuvers.begin(); };

        std::vector<Segment3d>::const_iterator maneuvers_end() const { return _maneuvers.end(); };

        /* Only for python interface */
        std::vector<double> start_times() const { return _start_times; };

        std::vector<double>::const_iterator start_times_begin() const { return _start_times.begin(); };

        std::vector<double>::const_iterator start_times_end() const { return _start_times.end(); };

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
        void freeze_before(double time) {
            ASSERT(_start_times.size() > 0);

            for (auto man_id = 0ul; man_id < size() - 1; ++man_id) {
                if (start_time(man_id) > time) {
                    freeze_before(man_id);
                    break;
                }
            }
        }

        /* Make all the maneuvers before 'man_index' unmodifiable. 'man_index' is still modifiable'*/
        void freeze_before(size_t man_index) {
            ASSERT(man_index <= size());
            insertion_range.start = man_index;
        }

        bool can_modify(size_t man_index) const {
            ASSERT(man_index < size());
            return insertion_range.contains(man_index);
        }

        bool can_insert_at(size_t man_index) const {
            ASSERT(man_index <= size());
            return insertion_range.contains(man_index);
        }

        std::vector<bool> modifiable() const {
            std::vector<bool> mod{};
            mod.reserve(size());

            for (size_t i = 0; i < size(); ++i) {
                mod.emplace_back(can_modify(i));
            }

            return mod;
        }

        /* Returns the part of a Trajectory within the TimewWindow as a new Trajectory. */
        Trajectory slice(TimeWindow tw) const {

            if (size() == 0) {
                // If the trajectory is void, a subtrajectory is the trajectory
                std::cerr << "Slicing an empty trajectory" << std::endl;
                return Trajectory(*this);
            }

            opt<size_t> new_traj_start_i = {};
            size_t new_traj_end_i = {};

            for (size_t i = 0; i < size(); ++i) {
                TimeWindow seg_range = TimeWindow(start_time(i), end_time(i));
                if (tw.intersects(seg_range)) {
                    if (!new_traj_start_i) {
                        new_traj_start_i = i;
                    }
                    new_traj_end_i = i;
                }
            }

            // When the new trajectory does not contain any vector
            if (!new_traj_start_i) {
                TrajectoryConfig new_conf = TrajectoryConfig(_conf.uav, tw.start, tw.end);
                return Trajectory(new_conf);
            }

            /* TODO: Start and end waypoints should be set to the position at tw.start and tw.end and not at the closest
             * segment. However, this requires to implement a function to get the position at any time in the trajectory.*/
            TrajectoryConfig new_conf = TrajectoryConfig(
                    _conf.uav, _maneuvers.at(*new_traj_start_i).start, _maneuvers.at(new_traj_end_i).end,
                    ((*new_traj_start_i) == 0) ? _conf.start_time : start_time(*new_traj_start_i),
                    (new_traj_end_i == (size() - 1)) ? _conf.max_flight_time : end_time(new_traj_end_i));

            Trajectory new_trajectory = Trajectory(new_conf);
            for (size_t i = *new_traj_start_i, j = new_trajectory.insertion_range_start();
                 i <= new_traj_end_i; ++i, ++j) {
                new_trajectory.insert_segment(_maneuvers[i], j);
            }

            return new_trajectory;
        };

        Trajectory with_longer_segments(double inc_length) {
            ASSERT(inc_length >= 0);
            Trajectory ext_traj = Trajectory(*this);
            for (size_t i = 0; i != size(); ++i) {
                if (ALMOST_EQUAL(ext_traj._maneuvers[i].length, 0)) { continue; }
                auto wps = Waypoint3d(
                        ext_traj._maneuvers[i].start.x - std::cos(ext_traj._maneuvers[i].start.dir) * inc_length / 2,
                        ext_traj._maneuvers[i].start.y - std::sin(ext_traj._maneuvers[i].start.dir) * inc_length / 2,
                        ext_traj._maneuvers[i].start.z, ext_traj._maneuvers[i].start.dir);
                auto wpe = Waypoint3d(
                        ext_traj._maneuvers[i].end.x + std::cos(ext_traj._maneuvers[i].end.dir) * inc_length / 2,
                        ext_traj._maneuvers[i].end.y + std::sin(ext_traj._maneuvers[i].end.dir) * inc_length / 2,
                        ext_traj._maneuvers[i].end.z, ext_traj._maneuvers[i].end.dir);
                ext_traj.replace_segment(i, Segment3d(wps, wpe));
            }
            return ext_traj;

        }

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::vector<Waypoint3d> as_waypoints() const {
            std::vector<Waypoint3d> waypoints;
            for (auto& man : _maneuvers) {
                waypoints.push_back(man.start);
                if (man.length > 0)
                    waypoints.push_back(man.end);
            }
            return waypoints;

        }

        /* Returns the trajectory as a set of waypoints.
         * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
         * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
        std::vector<Waypoint3d> sampled(const double step_size = 1) const {
            ASSERT(step_size > 0);
            std::vector<Waypoint3d> waypoints = as_waypoints();
            std::vector<Waypoint3d> sampled;
            if (waypoints.size() == 1)
                sampled.push_back(waypoints[0]);
            for (int i = 0; i < (int) waypoints.size() - 1; i++) {
                auto local_sampled = _conf.uav.path_sampling(waypoints[i], waypoints[i + 1], step_size);
                sampled.insert(sampled.end(), local_sampled.begin(), local_sampled.end());
            }
            return sampled;
        }

        /* Increase of length as result of inserting the given segment at the given position */
        double insertion_length_cost(size_t insert_loc, const Segment3d segment) const {
            if (size() == 0)
                return segment.length;
            else if (insert_loc == 0)
                return segment.length + _conf.uav.travel_distance(segment.end, _maneuvers[insert_loc].start);
            else if (insert_loc == size())
                return _conf.uav.travel_distance(_maneuvers[insert_loc - 1].end, segment.start) + segment.length;
            else {
                return _conf.uav.travel_distance(_maneuvers[insert_loc - 1].end, segment.start)
                       + segment.length
                       + _conf.uav.travel_distance(segment.end, _maneuvers[insert_loc].start)
                       - _conf.uav.travel_distance(_maneuvers[insert_loc - 1].end, _maneuvers[insert_loc].start);
            }
        }

        double insertion_duration_cost(size_t insert_loc, const Segment3d segment) const {
            auto increase_in_length = insertion_length_cost(insert_loc, segment);
            ASSERT(ALMOST_GREATER_EQUAL(increase_in_length, 0))
            return increase_in_length / _conf.uav.max_air_speed;
        }

        /* Decrease of length as result of removing the segment at the given position. */
        double removal_length_gain(size_t index) const {
            ASSERT(index >= 0 && index < size())
            const Segment3d segment = _maneuvers[index];
            if (index == 0)
                return segment.length + _conf.uav.travel_distance(segment.end, _maneuvers[index + 1].start);
            else if (index == size() - 1)
                return _conf.uav.travel_distance(_maneuvers[index - 1].end, segment.start) + segment.length;
            else {
                return _conf.uav.travel_distance(_maneuvers[index - 1].end, segment.start)
                       + segment.length
                       + _conf.uav.travel_distance(segment.end, _maneuvers[index + 1].start)
                       - _conf.uav.travel_distance(_maneuvers[index - 1].end, _maneuvers[index + 1].start);
            }
        }

        double removal_duration_gain(size_t index) const {
            return removal_length_gain(index) / _conf.uav.max_air_speed;
        }

        /* Computes the cost of replacing the N segments at [index, index+n] with the N segments given in parameter. */
        double replacement_length_cost(size_t index, const std::vector<Segment3d>& segments) const {
            return replacement_length_cost(index, segments.size(), segments);
        }

        /* Computes the cost of replacing the N segments at [index, index+n] with the N segments given in parameter. */
        double replacement_length_cost(size_t index, size_t n_replaced, const std::vector<Segment3d>& segments) const {
            ASSERT(n_replaced > 0);
            const unsigned long end_index = index + n_replaced - 1;
            ASSERT(index >= 0 && end_index < size())
            double cost =
                    segments_length(segments, 0, segments.size())
                    - segments_length(_maneuvers, index, n_replaced);
            if (index > 0)
                cost = cost
                       + _conf.uav.travel_distance(_maneuvers[index - 1].end, segments[0].start)
                       - _conf.uav.travel_distance(_maneuvers[index - 1].end, _maneuvers[index].start);
            if (end_index + 1 < size())
                cost = cost
                       + _conf.uav.travel_distance(segments[segments.size() - 1].end, _maneuvers[end_index + 1].start)
                       - _conf.uav.travel_distance(_maneuvers[end_index].end, _maneuvers[end_index + 1].start);

            return cost;
        }

        /* Increase in time (s) as a result of replacing the segment at the given index by the one provided.*/
        double replacement_duration_cost(size_t index, const Segment3d& segment) const {
            return replacement_duration_cost(index, std::vector<Segment3d>{segment});
        }

        /* Increase in time (s) as a result of replacing the N segments at the given index by the N segemnts provided.*/
        double replacement_duration_cost(size_t index, const std::vector<Segment3d>& segments) const {
            return replacement_length_cost(index, segments) / _conf.uav.max_air_speed;
        }

        /* Increase in time (s) as a result of replacing n segments at the given index by the N segments provided.*/
        double
        replacement_duration_cost(size_t index, size_t n_replaced, const std::vector<Segment3d>& segments) const {
            return replacement_length_cost(index, n_replaced, segments) / _conf.uav.max_air_speed;
        }

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
        Trajectory with_additional_segment(size_t index, const Segment3d& seg) const {
            Trajectory newTraj(*this);
            newTraj.insert_segment(seg, index);
            return newTraj;
        }

        /* In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
        Trajectory with_replaced_section(size_t index, const std::vector<Segment3d>& segments) const {
            Trajectory newTraj(*this);
            newTraj.replace_section(index, segments);
            return newTraj;
        }

        /* Adds segment to the end of the trajectory */
        void append_segment(const Segment3d& seg) {
            ASSERT(insertion_range.end > size())
            insert_segment(seg, size());

            check_validity();
        }

        /* Inserts the given segment at the given index */
        void insert_segment(const Segment3d& seg, size_t at_index) {
            ASSERT(at_index <= size())
            ASSERT(insertion_range_start() <= at_index && at_index <= insertion_range_end())
            const double start = at_index == 0 ? _conf.start_time :
                                 end_time(at_index - 1) +
                                 _conf.uav.travel_time(_maneuvers[at_index - 1].end, seg.start);

            const double added_delay = insertion_duration_cost(at_index, seg);
            ASSERT(ALMOST_GREATER_EQUAL(added_delay, 0))
            _maneuvers.insert(_maneuvers.begin() + at_index, seg);
            _start_times.insert(_start_times.begin() + at_index, start);

            for (size_t i = at_index + 1; i < size(); i++) {
                _start_times[i] += added_delay;
            }

            check_validity();
            insertion_range++;
        }

        /* Returns a new trajectory without the segment at the given index */
        Trajectory without_segment(size_t index) const {
            Trajectory newTraj(*this);
            newTraj.erase_segment(index);
            return newTraj;
        }

        /* Removes the segment at the given index. */
        void erase_all_modifiable_maneuvers() {
            while (modifiable_size() > 0) {
                erase_segment(first_modifiable_maneuver());
            }
        }

        /* Removes the segment at the given index. */
        void erase_segment(size_t at_index) {
            ASSERT(at_index >= 0 && at_index <= size())
            ASSERT(insertion_range.contains(at_index))
            const double gained_delay = removal_duration_gain(at_index);
            _maneuvers.erase(_maneuvers.begin() + at_index);
            _start_times.erase(_start_times.begin() + at_index);
            for (size_t i = at_index; i < size(); i++) {
                _start_times[i] -= gained_delay;
            }
            check_validity();
            insertion_range--;
        }

        void replace_segment(size_t at_index, const Segment3d& by_segment) {
            ASSERT(at_index >= 0 && at_index < size())
            erase_segment(at_index);
            insert_segment(by_segment, at_index);
            check_validity();
        }

        void replace_section(size_t index, const std::vector<Segment3d>& segments) {
            ASSERT(index >= 0 && index + segments.size() - 1 < size())
            for (size_t i = 0; i < segments.size(); i++) {
                erase_segment(index);
            }
            for (size_t i = 0; i < segments.size(); i++) {
                insert_segment(segments[i], index + i);
            }
            check_validity();
        }

        std::string to_string() const {
            std::stringstream repr;
            auto waypoints = as_waypoints();
            repr << "[";
            for (auto it = waypoints.begin(); it != waypoints.end(); it++)
                repr << (*it).to_string() << ", ";
            repr << "]";
            return repr.str();
        }

    private:
        TrajectoryConfig _conf;

        std::vector<Segment3d> _maneuvers;
        std::vector<double> _start_times;

        /* Boolean flag that is set to true once the trajectory is initialized.
         * Validity checks are only performed when this flag is true. */
        bool is_set_up = false;

        /* Range of vector<Segment3d> ids where segments can be modified, removed or inserted.
         * This is independent of the start and end points that can be fixed independently of this
         * setting */
        IndexRange insertion_range = IndexRange::unbounded(); // All by default

        /* Recomputes length from scratch and returns it. */
        double non_incremental_length() const {
            return segments_length(_maneuvers, 0, _maneuvers.size());
        }

        /* Functions that asserts invariants of a trajectory.
         * This is for debugging purpose only and the function content should be commented out. */
        void check_validity() const {
            if (is_set_up) {
//                ASSERT(traj.size() == start_times.size())
//        ASSERT(fabs(non_incremental_length() / conf.uav.max_air_speed - (end_time() - start_time())) < 0.001)
                for (size_t i = 0; i < size(); i++) {
                    ASSERT(ALMOST_GREATER_EQUAL(_start_times[i], _conf.start_time))
                }
                ASSERT(start_and_end_positions_respected())
            }
        }

        /* Converts a vector of waypoints to a vector of segments. */
        static std::vector<Segment3d> segments_from_waypoints(std::vector<Waypoint3d> waypoints) {
            std::vector<Segment3d> segments;
            std::transform(waypoints.begin(), waypoints.end(), segments.begin(),
                           [](const Waypoint3d& wp) { return Segment3d(wp); });
            return segments;
        }

        /* Computes the cost of a sub-trajectory composed of the given segments. */
        double segments_length(const std::vector<Segment3d>& segments, size_t start, size_t length) const {
            ASSERT(start + length <= segments.size());
            double cost = 0.;
            auto end_it = segments.begin() + start + length;
            for (auto it = segments.begin() + start; it != end_it; it++) {
                cost += it->length;
                if ((it + 1) != end_it)
                    cost += _conf.uav.travel_distance(it->end, (it + 1)->start);
            }
            return cost;
        }

        /* Computes the cost of a sub-trajectory composed of the given segments. */
        double segments_length(const std::vector<TimedManeuver>& maneuvers, size_t start, size_t length) const {
            ASSERT(start + length <= maneuvers.size());
            double cost = 0.;
            auto end_it = maneuvers.begin() + start + length;
            for (auto it = maneuvers.begin() + start; it != end_it; it++) {
                cost += it->maneuver.length;
                if ((it + 1) != end_it)
                    cost += _conf.uav.travel_distance(it->maneuver.end, (it + 1)->maneuver.start);
            }
            return cost;
        }
    };
}
#endif //PLANNING_CPP_TRAJECTORY_H
