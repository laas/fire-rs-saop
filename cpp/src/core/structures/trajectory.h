#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#import <string>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <memory>
#include "../../ext/dubins.h"
#include "waypoint.h"
#include "uav.h"
#include "../../utils.h"
#include "../../ext/optional.h"

struct TrajectoryConfig {
    const UAV uav;
    const double start_time;
    const opt<Waypoint3d> start_position;
    const opt<Waypoint3d> end_position;
    const double max_flight_time;

    TrajectoryConfig(const UAV& uav, double start_time = 0, double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position((opt<Waypoint3d>){}), end_position((opt<Waypoint3d>){}), max_flight_time(max_flight_time) {}

    TrajectoryConfig(const UAV& uav,
                               const Waypoint3d& start_position,
                               double start_time = 0,
                               double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position(start_position), end_position(opt<Waypoint3d>()), max_flight_time(std::numeric_limits<double>::max()) {}

    TrajectoryConfig(const UAV& uav,
                               const Waypoint3d& start_position,
                               const Waypoint3d& end_position,
                               double start_time = 0,
                               double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position(start_position), end_position(end_position),
              max_flight_time(max_flight_time) {}
};

class Trajectory {
public:
    TrajectoryConfig conf;
    std::vector<Segment3d> traj;
    std::vector<double> start_times;

    Trajectory(const Trajectory& trajectory) = default;

    /** Empty trajectory constructor */
    explicit Trajectory(const TrajectoryConfig& _config)
            : conf(_config)
    {
        if(conf.start_position)
            append_segment(Segment3d(*conf.start_position));
        if(conf.end_position)
            append_segment(Segment3d(*conf.end_position));
        is_set_up = true;
        check_validity();
    }

    /** Number of segments in the trajectory. */
    size_t size() const { return traj.size(); }

    /** Time (s) at which the trajectory starts. */
    double start_time() const { return conf.start_time; }

    /** Time (s) at which the trajectory ends. */
    double end_time() const { return traj.size() == 0 ? start_time() : end_time(traj.size()-1); }

    /** Time (s) at which the UAV reaches the start waypoint of a given segment. */
    double start_time(size_t segment_index) const {
        ASSERT(traj.size() == start_times.size())
        ASSERT(segment_index >= 0 && segment_index < traj.size())
        return start_times[segment_index];
    }

    /** Time (s) at which the UAV reaches the end waypoint of a given segment. */
    double end_time(size_t segment_index) const {
        ASSERT(traj.size() == start_times.size())
        ASSERT(segment_index >= 0 && segment_index < traj.size())
        return start_time(segment_index) + traj[segment_index].length / conf.uav.max_air_speed;
    }

    /** Duration (s) of the path. */
    double duration() const {
        return end_time() - start_time();
    }

    /** Length (m) of the path. */
    double length() const {
        return conf.uav.max_air_speed * duration();
    }

    /** Returns true if the first/last segments matches the ones given in the configuration of the trajectory. */
    bool start_and_end_positions_respected() const {
        const bool start_valid = !conf.start_position ||
                                 (traj.size() >= 1 && traj[0].start == *conf.start_position &&
                                  traj[0].end == *conf.start_position && traj[0].length == 0);
        const size_t last_index = traj.size()-1;
        const bool end_valid = !conf.end_position ||
                               (traj.size() >= 1 && traj[last_index].start == *conf.end_position &&
                                traj[last_index].end == *conf.end_position && traj[last_index].length == 0);
        return start_valid && end_valid;
    }

    /** Returns true if this trajectory does not go over the maximum flight time given in configuration. */
    bool has_valid_flight_time() const {
        return ALMOST_LESSER_EQUAL(duration(), conf.max_flight_time);
    }

    /** Index of the first modifiable segment. */
    size_t first_modifiable() const {
        return conf.start_position ? 1 : 0;
    }

    /** Index of the last modifiable segment */
    size_t last_modifiable() const {
        return conf.end_position ? traj.size()-2 : traj.size() -1;
    }

    /* Random segment index selected among modifiable segments */
    inline opt<size_t> get_random_modifiable_id() const {
        if (first_modifiable() > last_modifiable()) {
            return {};
        }
        return rand(first_modifiable(), last_modifiable() + 1);
    }

    /** Accesses the index-th segment of the trajectory */
    const Segment3d& operator[] (size_t index) const { return traj[index]; }

    /** Returns the trajectory as a set of waypoints.
     * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
     * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
    std::vector<Waypoint3d> as_waypoints() const {
        std::vector<Waypoint3d> waypoints;
        for(auto& segment : traj) {
            waypoints.push_back(segment.start);
            if(segment.length > 0)
                waypoints.push_back(segment.end);
        }
        return waypoints;

    }

    /** Returns the trajectory as a set of waypoints.
    * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
    * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
    std::vector<Waypoint3d> sampled(const double step_size = 1) const {
        ASSERT(step_size > 0);
        std::vector<Waypoint3d> waypoints = as_waypoints();
        std::vector<Waypoint3d> sampled;
        if(waypoints.size() == 1)
            sampled.push_back(waypoints[0]);
        for(int i=0; i<(int)waypoints.size()-1; i++) {
            auto local_sampled = conf.uav.path_sampling(waypoints[i], waypoints[i+1], step_size);
            sampled.insert(sampled.end(), local_sampled.begin(), local_sampled.end());
        }
        return sampled;

    }

    /** Returns a new Trajectory with the given segment appended. */
    Trajectory with_segment_at_end(const Segment3d& segment) const {
        return with_additional_segment(traj.size(), segment);
    }

    /** Returns a new trajectory with the given waypoint appended (as a segment of length 0) */
    Trajectory with_waypoint_at_end(Waypoint3d& waypoint) const {
        Segment3d s(waypoint);
        return with_segment_at_end(s);
    }

    /** Increase of length as result of inserting the given segment at the given position */
    double insertion_length_cost(size_t insert_loc, const Segment3d segment) const {
        if(traj.size() == 0)
            return segment.length;
        else if(insert_loc == 0)
            return segment.length + conf.uav.travel_distance(segment.end, traj[insert_loc].start);
        else if(insert_loc == traj.size())
            return conf.uav.travel_distance(traj[insert_loc - 1].end, segment.start) + segment.length;
        else {
            return conf.uav.travel_distance(traj[insert_loc - 1].end, segment.start)
                   + segment.length
                   + conf.uav.travel_distance(segment.end, traj[insert_loc].start)
                   - conf.uav.travel_distance(traj[insert_loc - 1].end, traj[insert_loc].start);
        }
    }
    double insertion_duration_cost(size_t insert_loc, const Segment3d segment) const {
        auto increase_in_length =  insertion_length_cost(insert_loc, segment);
        ASSERT(increase_in_length >= 0)
        return increase_in_length / conf.uav.max_air_speed;
    }

    /** Decrease of length as result of removing the segment at the given position. */
    double removal_length_gain(size_t index) const {
        ASSERT(index >= 0 && index < traj.size())
        const Segment3d segment = traj[index];
        if(index == 0)
            return segment.length + conf.uav.travel_distance(segment.end, traj[index+1].start);
        else if(index == traj.size()-1)
            return conf.uav.travel_distance(traj[index - 1].end, segment.start) + segment.length;
        else {
            return conf.uav.travel_distance(traj[index - 1].end, segment.start)
                   + segment.length
                   + conf.uav.travel_distance(segment.end, traj[index+1].start)
                   - conf.uav.travel_distance(traj[index - 1].end, traj[index+1].start);
        }
    }
    double removal_duration_gain(size_t index) const {
        return removal_length_gain(index) / conf.uav.max_air_speed;
    }

    /** Computes the cost of replacing the N segments at [index, index+n] with the N segments given in parameter. */
    double replacement_length_cost(size_t index, const std::vector<Segment3d>& segments) const {
        return replacement_length_cost(index, segments.size(), segments);
    }

    /** Computes the cost of replacing the N segments at [index, index+n] with the N segments given in parameter. */
    double replacement_length_cost(size_t index, size_t n_replaced, const std::vector<Segment3d>& segments) const {
        ASSERT(n_replaced > 0);
        const unsigned long end_index = index + n_replaced - 1;
        ASSERT(index >= 0 && end_index < traj.size())
        double cost =
                segments_length(segments, 0, segments.size())
                - segments_length(traj, index, n_replaced);
        if(index > 0)
            cost = cost
                   + conf.uav.travel_distance(traj[index-1].end, segments[0].start)
                   - conf.uav.travel_distance(traj[index-1].end, traj[index].start);
        if(end_index+1 < traj.size())
            cost = cost
                   + conf.uav.travel_distance(segments[segments.size()-1].end, traj[end_index+1].start)
                   - conf.uav.travel_distance(traj[end_index].end, traj[end_index+1].start);

        return cost;
    }

    /** Increase in time (s) as a result of replacing the segment at the given index by the one provided.*/
    double replacement_duration_cost(size_t index, const Segment3d& segment) const {
        return replacement_duration_cost(index, std::vector<Segment3d>{ segment });
    }

    /** Increase in time (s) as a result of replacing the N segments at the given index by the N segemnts provided.*/
    double replacement_duration_cost(size_t index, const std::vector<Segment3d>& segments) const {
        return replacement_length_cost(index, segments) / conf.uav.max_air_speed;
    }

    /** Increase in time (s) as a result of replacing n segments at the given index by the N segments provided.*/
    double replacement_duration_cost(size_t index, size_t n_replaced, const std::vector<Segment3d>& segments) const {
        return replacement_length_cost(index, n_replaced, segments) / conf.uav.max_air_speed;
    }

    /** Returns a new trajectory with an additional segment at the given index */
    Trajectory with_additional_segment(size_t index, const Segment3d& seg) const {
        Trajectory newTraj(*this);
        newTraj.insert_segment(seg, index);
        return newTraj;
    }

    /** Adds segment to the end of the trajectory */
    void append_segment(const Segment3d& seg) {
        insert_segment(seg, traj.size());
        ASSERT(traj.size() == start_times.size())
    }

    /** Inserts the given segment at the given index */
    void insert_segment(const Segment3d& seg, size_t at_index) {
        ASSERT(at_index >= 0 && at_index <= traj.size())
        const double start = at_index == 0 ?
                          conf.start_time :
                          end_time(at_index-1) + conf.uav.travel_time(traj[at_index-1].end, seg.start);

        const double added_delay = insertion_duration_cost(at_index, seg);
        ASSERT(added_delay >= 0)
        traj.insert(traj.begin()+at_index, seg);
        start_times.insert(start_times.begin()+at_index, start);

        for(size_t i=at_index+1; i<start_times.size(); i++) {
            start_times[i] += added_delay;
        }

        check_validity();
    }

    /** Returns a new trajectory without the segment at the given index */
    Trajectory without_segment(size_t index) const {
        Trajectory newTraj(*this);
        newTraj.erase_segment(index);
        return newTraj;
    }

    /** Removes the segment at the given index. */
    void erase_segment(size_t at_index) {
        const double gained_delay = removal_duration_gain(at_index);
        traj.erase(traj.begin() + at_index);
        start_times.erase(start_times.begin() + at_index);
        for(size_t i=at_index; i<start_times.size(); i++) {
            start_times[i] -= gained_delay;
        }
        check_validity();
    }

    /** In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
    Trajectory with_replaced_section(size_t index, const std::vector<Segment3d>& segments) const {
        Trajectory newTraj(*this);
        newTraj.replace_section(index, segments);
        return newTraj;
    }

    void replace_segment(size_t at_index, const Segment3d& by_segment) {
        ASSERT(at_index >= 0 && at_index < traj.size())
        erase_segment(at_index);
        insert_segment(by_segment, at_index);
        check_validity();
    }

    void replace_section(size_t index, const std::vector<Segment3d>& segments) {
        ASSERT(index >= 0 && index+segments.size()-1 < traj.size())
        for(size_t i=0; i < segments.size(); i++) {
            erase_segment(index);
        }
        for(size_t i=0; i < segments.size(); i++) {
            insert_segment(segments[i], index+i);
        }
        check_validity();
    }

    std::string to_string() const {
        std::stringstream repr;
        auto waypoints = as_waypoints();
        repr << "[";
        for(auto it=waypoints.begin(); it!=waypoints.end(); it++)
            repr << (*it).to_string() << ", ";
        repr << "]";
        return repr.str();
    }

private:
    /** Boolean flag that is set to true once the trajectory is initialized.
     * Validity checks are only performed when this flag is true. */
    bool is_set_up = false;

    /** Recomputes length from scratch and returns it. */
    double non_incremental_length() const {
        return segments_length(traj, 0, traj.size());
    }

    /** Functions that asserts invariants of a trajectory.
     * This is for debugging purpose only and the function content should be commented out. */
    void check_validity() const {
        if(is_set_up) {
            ASSERT(traj.size() == start_times.size())
//        ASSERT(fabs(non_incremental_length() / conf.uav.max_air_speed - (end_time() - start_time())) < 0.001)
            for (size_t i = 0; i < start_times.size(); i++) {
                ASSERT(ALMOST_GREATER_EQUAL(start_times[i], conf.start_time))
            }
            ASSERT(start_and_end_positions_respected())
        }
    }

    /** Converts a vector of waypoints to a vector of segments. */
    static std::vector<Segment3d> segments_from_waypoints(std::vector<Waypoint3d> waypoints) {
        std::vector<Segment3d> segments;
        std::transform(waypoints.begin(), waypoints.end(), segments.begin(),
                       [](const Waypoint3d& wp) { return Segment3d(wp); });
        return segments;
    }

    /** Computes the cost of a sub-trajectory composed of the given segments. */
    double segments_length(const std::vector<Segment3d>& segments, size_t start, size_t length) const {
        ASSERT(start+length <= segments.size());
        double cost = 0.;
        auto end_it = segments.begin() + start + length;
        for(auto it=segments.begin()+start; it!=end_it; it++) {
            cost += it->length;
            if((it+1) != end_it)
                cost += conf.uav.travel_distance(it->end, (it+1)->start);
        }
        return cost;
    }
};

#endif //PLANNING_CPP_TRAJECTORY_H
