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

#include "trajectory.hpp"

namespace SAOP {
    Trajectory::Trajectory(const TrajectoryConfig& config)
            : config(config) {
        if (config.start_position) {
            std::string takeoff_name = "takeoff" + std::to_string(UNIQUE_MAN_N);
            UNIQUE_MAN_N++;
            append_segment(Segment3d(*config.start_position), takeoff_name);
            insertion_range = IndexRange::end_unbounded(1);
        }

        if (config.end_position) {
            std::string land_name = "land" + std::to_string(UNIQUE_MAN_N);
            UNIQUE_MAN_N++;
            append_segment(Segment3d(*config.end_position), land_name);
            insertion_range = insertion_range.intersection_with(IndexRange::start_unbounded(size() - 1));
        }
        is_set_up = true;
        check_validity();
    }

    void Trajectory::freeze_before(double time) {
        ASSERT(_start_times.size() > 0);

        for (auto man_id = 0ul; man_id < size() - 1; ++man_id) {
            if (start_time(man_id) > time) {
                freeze_before(man_id);
                break;
            }
        }
    }

    void Trajectory::freeze_before(size_t man_index) {
        ASSERT(man_index <= size());
        insertion_range.start = man_index;
    }

    bool Trajectory::can_modify(size_t man_index) const {
        ASSERT(man_index < size());
        return insertion_range.contains(man_index);
    }

    bool Trajectory::can_insert_at(size_t man_index) const {
        ASSERT(man_index <= size());
        return insertion_range.contains(man_index);
    }

    std::vector<bool> Trajectory::modifiable() const {
        std::vector<bool> mod{};
        mod.reserve(size());

        for (size_t i = 0; i < size(); ++i) {
            mod.emplace_back(can_modify(i));
        }

        return mod;
    }

    Trajectory Trajectory::slice(TimeWindow tw) const {

        if (size() == 0) {
            // If the trajectory is void, a subtrajectory is the trajectory
            BOOST_LOG_TRIVIAL(debug) << "Slicing an empty trajectory" << std::endl;
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
            TrajectoryConfig new_conf = TrajectoryConfig(config.uav, tw.start, tw.end);
            return Trajectory(new_conf);
        }

        /* TODO: Start and end waypoints should be set to the position at tw.start and tw.end and not at the closest
         * segment. However, this requires to implement a function to get the position at any time in the trajectory.*/
        TrajectoryConfig new_conf = TrajectoryConfig(
                config.uav, _maneuvers.at(*new_traj_start_i).start, _maneuvers.at(new_traj_end_i).end,
                ((*new_traj_start_i) == 0) ? config.start_time : start_time(*new_traj_start_i),
                (new_traj_end_i == (size() - 1)) ? config.max_flight_time : end_time(new_traj_end_i));

        Trajectory new_trajectory = Trajectory(new_conf);
        for (size_t i = *new_traj_start_i, j = new_trajectory.insertion_range_start();
             i <= new_traj_end_i; ++i, ++j) {
            new_trajectory.insert_segment(_maneuvers[i], j);
        }

        return new_trajectory;
    }

    Trajectory Trajectory::with_longer_segments(double inc_length) {
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

    std::vector<Waypoint3d> Trajectory::as_waypoints() const {
        std::vector<Waypoint3d> waypoints;
        for (auto& man : _maneuvers) {
            waypoints.push_back(man.start);
            if (man.length > 0)
                waypoints.push_back(man.end);
        }
        return waypoints;

    }

    std::pair<std::vector<Waypoint3d>, std::vector<double>> Trajectory::as_waypoints_with_time() const {
        std::vector<Waypoint3d> waypoints = {};
        std::vector<double> time = {};
        for (auto i = 0ul; i < _maneuvers.size(); ++i) {
            waypoints.push_back(_maneuvers[i].start);
            time.push_back(_start_times[i]);
            if (_maneuvers[i].length > 0) {
                waypoints.push_back(_maneuvers[i].end);
                time.push_back(end_time(i));
            }
        }
        return {waypoints, time};
    }

    std::vector<Waypoint3d> Trajectory::sampled(const double step_size) const {
        ASSERT(step_size > 0);
        std::vector<Waypoint3d> waypoints = as_waypoints();
        std::vector<Waypoint3d> sampled;
        if (waypoints.size() == 1)
            sampled.push_back(waypoints[0]);
        for (int i = 0; i < (int) waypoints.size() - 1; i++) {
            auto local_sampled = config.uav.path_sampling(waypoints[i], waypoints[i + 1], config.wind, step_size);
            sampled.insert(sampled.end(), local_sampled.begin(), local_sampled.end());
        }
        return sampled;
    }

    std::pair<std::vector<Waypoint3d>, std::vector<double>> Trajectory::sampled_with_time(double step_size) const {
        ASSERT(step_size > 0);

        auto waypoints_time = as_waypoints_with_time();

        if (std::get<0>(waypoints_time).empty()) {
            return {};
        }

        std::vector<Waypoint3d> sampled;
        std::vector<double> time;

        if (std::get<0>(waypoints_time).size() == 1) {
            sampled.push_back(std::get<0>(waypoints_time)[0]);
            time.push_back(std::get<1>(waypoints_time)[0]);
        }
        // Time of the first waypoint -> start time
        double cumulated_travel_time = std::get<1>(waypoints_time)[0];

        for (int i = 0; i < (int) std::get<0>(waypoints_time).size() - 1; i++) {
            // Sample trajectory between waypoints
            auto local_sampled = config.uav.path_sampling_with_time(std::get<0>(waypoints_time)[i],
                                                                    std::get<0>(waypoints_time)[i + 1], config.wind,
                                                                    step_size, cumulated_travel_time);
            sampled.insert(sampled.end(), std::get<0>(local_sampled).begin(), std::get<0>(local_sampled).end());
            time.insert(time.end(), std::get<1>(local_sampled).begin(), std::get<1>(local_sampled).end());
            if (!time.empty()) {
                cumulated_travel_time = time.back();
            }
        }
        return {sampled, time};
    }

    std::pair<std::vector<Waypoint3d>, std::vector<double>>
    Trajectory::sampled_with_time(TimeWindow time_range, double step_size) const {
        ASSERT(step_size > 0);

        auto waypoints_time = as_waypoints_with_time();

        std::vector<Waypoint3d> sampled;
        std::vector<double> time;

        if (std::get<0>(waypoints_time).size() == 1 && time_range.contains(std::get<1>(waypoints_time)[0])) {
            sampled.push_back(std::get<0>(waypoints_time)[0]);
            time.push_back(std::get<1>(waypoints_time)[0]);
        }
        // Time of the first waypoint -> start time
        double cumulated_travel_time = std::get<1>(waypoints_time)[0];

        for (size_t i = 0; i < std::get<0>(waypoints_time).size() - 1; i++) {

            // Sample trajectory between waypoints that are at least partially on the time range
            if (time_range.contains(std::get<1>(waypoints_time)[i]) ||
                time_range.contains(std::get<1>(waypoints_time)[i + 1])) {
                auto local_sampled = config.uav.path_sampling_with_time(std::get<0>(waypoints_time)[i],
                                                                        std::get<0>(waypoints_time)[i + 1],
                                                                        config.wind, step_size,
                                                                        cumulated_travel_time);
                auto l_sam_wp = std::get<0>(local_sampled).begin();
                auto l_sam_t = std::get<1>(local_sampled).begin();

                while (l_sam_wp != std::get<0>(local_sampled).end() ||
                       l_sam_t != std::get<1>(local_sampled).end()) {
                    // Only add to the final vector those samples that are strictly in the time range
                    if (time_range.contains(*l_sam_t)) {
                        sampled.emplace_back(*l_sam_wp);
                        time.emplace_back(*l_sam_t);
                    }
                    ++l_sam_wp;
                    ++l_sam_t;
                }
                if (time.size() > 0) {
                    cumulated_travel_time = time.back();
                } else {
                    cumulated_travel_time = std::get<1>(waypoints_time)[i + 1];
                }
            } else {
                cumulated_travel_time = std::get<1>(waypoints_time)[i + 1];
            }
        }
        return {sampled, time};
    }

    double Trajectory::insertion_duration_cost(size_t insert_loc, const Segment3d segment) const {
        double delta_time;
        if (size() == 0)
            delta_time = config.uav.travel_time(segment, config.wind);
        else if (insert_loc == 0)
            delta_time = config.uav.travel_time(segment, config.wind)
                         + config.uav.travel_time(segment.end, _maneuvers[insert_loc].start, config.wind);
        else if (insert_loc == size())
            delta_time = config.uav.travel_time(_maneuvers[insert_loc - 1].end, segment.start, config.wind)
                         + config.uav.travel_time(segment, config.wind);
        else {
            delta_time = config.uav.travel_time(_maneuvers[insert_loc - 1].end, segment.start, config.wind)
                         + config.uav.travel_time(segment, config.wind)
                         + config.uav.travel_time(segment.end, _maneuvers[insert_loc].start, config.wind)
                         - config.uav.travel_time(_maneuvers[insert_loc - 1].end, _maneuvers[insert_loc].start,
                                                  config.wind);
        }

        if (delta_time < 0) {
            BOOST_LOG_TRIVIAL(warning) << "Trajectory::insert_segment(" << segment << ", " << insert_loc
                                       << ") . Added delay is negative " << delta_time;
        }

        return delta_time;

    }

    double Trajectory::removal_duration_gain(size_t index) const {
        ASSERT(index < size());
        const Segment3d segment = _maneuvers[index];
        if (index == 0)
            return config.uav.travel_time(segment, config.wind)
                   + config.uav.travel_time(segment.end, _maneuvers[index + 1].start, config.wind);
        else if (index == size() - 1)
            return config.uav.travel_time(_maneuvers[index - 1].end, segment.start, config.wind)
                   + config.uav.travel_time(segment, config.wind);
        else {
            return config.uav.travel_time(_maneuvers[index - 1].end, segment.start, config.wind)
                   + config.uav.travel_time(segment, config.wind)
                   + config.uav.travel_time(segment.end, _maneuvers[index + 1].start, config.wind)
                   - config.uav.travel_time(_maneuvers[index - 1].end, _maneuvers[index + 1].start, config.wind);
        }
    }

    double Trajectory::replacement_duration_cost(size_t index, size_t n_replaced,
                                                 const std::vector<Segment3d>& segments) const {
        ASSERT(n_replaced > 0);
        const unsigned long end_index = index + n_replaced - 1;
        ASSERT(end_index < size());
        double duration =
                segments_duration(segments, 0, segments.size())
                - segments_duration(_maneuvers, index, n_replaced);
        if (index > 0)
            duration = duration
                       + config.uav.travel_time(_maneuvers[index - 1].end, segments[0].start, config.wind)
                       - config.uav.travel_time(_maneuvers[index - 1].end, _maneuvers[index].start, config.wind);
        if (end_index + 1 < size())
            duration = duration
                       + config.uav.travel_time(segments[segments.size() - 1].end, _maneuvers[end_index + 1].start,
                                                config.wind)
                       - config.uav.travel_time(_maneuvers[end_index].end, _maneuvers[end_index + 1].start,
                                                config.wind);

        return duration;
    }

    Trajectory Trajectory::with_additional_segment(size_t index, const Segment3d& seg) const {
        Trajectory newTraj(*this);
        newTraj.insert_segment(seg, index);
        return newTraj;
    }

    Trajectory Trajectory::with_replaced_section(size_t index, const std::vector<Segment3d>& segments) const {
        Trajectory newTraj(*this);
        newTraj.replace_section(index, segments);
        return newTraj;
    }

    void Trajectory::append_segment(const Segment3d& seg) {
        ASSERT(insertion_range.end > size());
        insert_segment(seg, size());

        check_validity();
    }

    void Trajectory::append_segment(const Segment3d& seg, std::string name) {
        ASSERT(insertion_range.end > size());
        insert_segment(seg, size(), name);

        check_validity();
    }

    void Trajectory::insert_segment(const Segment3d& seg, size_t at_index) {
        std::string name = "wp" + std::to_string(UNIQUE_MAN_N);
        UNIQUE_MAN_N++;
        insert_segment(seg, at_index,name);
    }

    void Trajectory::insert_segment(const Segment3d& seg, size_t at_index, std::string name) {
        ASSERT(at_index <= size());
        ASSERT(insertion_range_start() <= at_index && at_index <= insertion_range_end());
        const double start = at_index == 0 ? config.start_time :
                             end_time(at_index - 1) +
                             config.uav.travel_time(_maneuvers[at_index - 1].end, seg.start, config.wind);

        const double added_delay = insertion_duration_cost(at_index, seg);
        if (!ALMOST_GREATER_EQUAL(added_delay, 0)) {
            BOOST_LOG_TRIVIAL(warning) << "Trajectory::insert_segment(" << seg << ", " << at_index
                                       << ") . Added delay is negative " << added_delay << std::endl;
//                insertion_duration_cost(at_index, seg);
        }
        ASSERT(added_delay < std::numeric_limits<double>::infinity());
        _maneuvers.insert(_maneuvers.begin() + at_index, seg);
        _start_times.insert(_start_times.begin() + at_index, start);
        _man_names.insert(_man_names.begin() + at_index, name);
        for (size_t i = at_index + 1; i < size(); i++) {
            _start_times[i] += added_delay;
        }

        check_validity();
        insertion_range++;
    }

    Trajectory Trajectory::without_segment(size_t index) const {
        Trajectory newTraj(*this);
        newTraj.erase_segment(index);
        return newTraj;
    }

    void Trajectory::erase_all_modifiable_maneuvers() {
        while (modifiable_size() > 0) {
            erase_segment(first_modifiable_maneuver());
        }
    }

    void Trajectory::erase_segment(size_t at_index) {
        ASSERT(at_index <= size());
        ASSERT(insertion_range.contains(at_index));
        const double gained_delay = removal_duration_gain(at_index);
        _maneuvers.erase(_maneuvers.begin() + at_index);
        _start_times.erase(_start_times.begin() + at_index);
        _man_names.erase(_man_names.begin() + at_index);
        for (size_t i = at_index; i < size(); i++) {
            _start_times[i] -= gained_delay;
        }
        check_validity();
        insertion_range--;
    }

    void Trajectory::replace_segment(size_t at_index, const Segment3d& by_segment) {
        ASSERT(at_index < size());
        std::string keep_name = _man_names[at_index];
        erase_segment(at_index);
        insert_segment(by_segment, at_index, keep_name);
        check_validity();
    }

    void Trajectory::replace_section(size_t index, const std::vector<Segment3d>& segments) {
        ASSERT(index + segments.size() - 1 < size());
        std::vector<std::string> keep_names = {};
        for (size_t i = 0; i < segments.size(); i++) {
            keep_names.emplace_back(_man_names[i]);
            erase_segment(index);
        }
        for (size_t i = 0; i < segments.size(); i++) {
            insert_segment(segments[i], index + i, _man_names[i]);
        }
        check_validity();
    }

    std::string Trajectory::to_string() const {
        std::stringstream repr;
        auto waypoints = as_waypoints();
        repr << "[";
        for (auto it = waypoints.begin(); it != waypoints.end(); it++)
            repr << (*it).to_string() << ", ";
        repr << "]";
        return repr.str();
    }

    void Trajectory::check_validity() const {
        if (is_set_up) {
//                ASSERT(traj.size() == start_times.size())
//        ASSERT(fabs(non_incremental_length() / conf.uav.max_air_speed() - (end_time() - start_time())) < 0.001)
            for (size_t i = 0; i < size(); i++) {
                ASSERT(ALMOST_GREATER_EQUAL(_start_times[i], config.start_time));
            }
            ASSERT(start_and_end_positions_respected());
        }
    }

    std::vector<Segment3d> Trajectory::segments_from_waypoints(std::vector<Waypoint3d> waypoints) {
        std::vector<Segment3d> segments;
        std::transform(waypoints.begin(), waypoints.end(), segments.begin(),
                       [](const Waypoint3d& wp) { return Segment3d(wp); });
        return segments;
    }

    double Trajectory::segments_duration(const std::vector<Segment3d>& segments, size_t start, size_t length) const {
        ASSERT(start + length <= segments.size());
        double duration = 0.;
        auto end_it = segments.begin() + start + length;
        for (auto it = segments.begin() + start; it != end_it; it++) {
            duration += config.uav.travel_time(*it, config.wind);
            if ((it + 1) != end_it)
                duration += config.uav.travel_time(it->end, (it + 1)->start, config.wind);
        }
        return duration;
    }


}