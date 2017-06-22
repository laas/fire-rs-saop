#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#import <string>
#include <sstream>
#include <algorithm>
#include <cassert>
#include <memory>
#include "ext/dubins.h"
#include "waypoint.h"
#include "uav.h"
#include "debug.h"
#include "ext/optional.h"

struct Segment {
    Waypoint start;
    Waypoint end;
    double length;

    constexpr Segment(const Waypoint& wp1) : start(wp1), end(wp1), length(0) {}
    constexpr Segment(const Waypoint& wp1, const Waypoint& wp2)
            : start(wp1), end(wp2), length(sqrt(pow(wp1.x-wp2.x, 2) + pow(wp1.y-wp2.y, 2))) {}
    constexpr Segment(const Waypoint& wp1, const double length)
            : start(wp1), end(Waypoint(wp1.x +cos(wp1.dir)*length, wp1.y + sin(wp1.dir)*length, wp1.dir)), length(length) {}

    friend std::ostream& operator<< (std::ostream& stream, const Segment& s) {
        return stream << "<" << s.start << ", " << s.length << ">";
    }
};

struct TrajectoryConfig {
    const UAV uav;
    const double start_time;
    const opt<Waypoint> start_position;
    const opt<Waypoint> end_position;
    const double max_flight_time;

    constexpr TrajectoryConfig(const UAV& uav, double start_time = 0, double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position({}), end_position({}), max_flight_time(max_flight_time) {}

    constexpr TrajectoryConfig(const UAV& uav,
                               const Waypoint& start_position,
                               double start_time = 0,
                               double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position(start_position), end_position({}), max_flight_time(std::numeric_limits<double>::max()) {}

    constexpr TrajectoryConfig(const UAV& uav,
                               const Waypoint& start_position,
                               const Waypoint& end_position,
                               double start_time = 0,
                               double max_flight_time = std::numeric_limits<double>::max())
            : uav(uav), start_time(start_time), start_position(start_position), end_position(end_position),
              max_flight_time(max_flight_time) {}
};

class Trajectory {
public:
    std::shared_ptr<TrajectoryConfig> conf;
    std::vector<Segment> traj;
    std::vector<double> start_times;

    Trajectory(const Trajectory& trajectory) = default;

public:
    double start_time() const { return conf->start_time; }
    double end_time() const { return traj.size() == 0 ? start_time() : end_time(traj.size()-1); }

    double start_time(size_t segment_index) const {
        ASSERT(traj.size() == start_times.size())
        ASSERT(segment_index >= 0 && segment_index < traj.size())
        return start_times[segment_index];
    }

    double end_time(size_t segment_index) const {
        ASSERT(traj.size() == start_times.size())
        ASSERT(segment_index >= 0 && segment_index < traj.size())
        return start_time(segment_index) + traj[segment_index].length / conf->uav.max_air_speed;
    }

    double duration() const {
        return end_time() - start_time();
    }

    /** Empty trajectory constructor */
    Trajectory(std::shared_ptr<TrajectoryConfig> _config)
            : conf(_config)
    {
        if(conf->start_position)
            append_segment(Segment(*conf->start_position));
        if(conf->end_position)
            append_segment(Segment(*conf->end_position));
    }

    Trajectory(const TrajectoryConfig& c) : Trajectory(std::make_shared<TrajectoryConfig>(c)) {
        ASSERT(matches_configuration());
    }

    Trajectory(std::shared_ptr<TrajectoryConfig> conf, std::vector<Segment> traj)
            : conf(conf)
    {
        for(auto it=traj.begin(); it!=traj.end(); it++)
            append_segment(*it);
        ASSERT(matches_configuration());
    }

    Trajectory(std::shared_ptr<TrajectoryConfig> conf, std::vector<Waypoint> waypoints)
            : conf(conf)
    {
        for(auto it=waypoints.begin(); it!=waypoints.end(); it++)
            append_segment(Segment(*it));
        ASSERT(matches_configuration());
    }

    /** Accesses the index-th segment of the trajectory */
    const Segment& operator[] (unsigned long index) const { return traj[index]; }

    /** Returns the trajectory as a set of waypoints.
     * If step_size < 0, only waypoints corresponding to start/end of segments are returned.
     * Otherwise, there will one waypoint every 'step_size' distance units of the path. */
    std::vector<Waypoint> as_waypoints(const double step_size = -1) const {
        std::vector<Waypoint> waypoints;
        for(auto it = traj.begin(); it!=traj.end(); it++) {
            waypoints.push_back(it->start);
            if(it->length > 0)
                waypoints.push_back(it->end);
        }
        if(step_size < 0) {
            return waypoints;
        } else {
            std::vector<Waypoint> sampled;
            if(waypoints.size() == 1)
                sampled.push_back(waypoints[0]);
            for(unsigned i=0; i<waypoints.size()-1; i++) {
                auto local_sampled = conf->uav.path_sampling(waypoints[i], waypoints[i+1], step_size);
                sampled.insert(sampled.end(), local_sampled.begin(), local_sampled.end());
            }
            return sampled;
        }
    }

    /** Returns a new Trajectory with the given segment appended. */
    Trajectory with_segment_at_end(const Segment& segment) const {
        return with_additional_segment(traj.size(), segment);
    }

    /** Returns a new trajectory with the given waypoint appended (as a segment of length 0) */
    Trajectory with_waypoint_at_end(Waypoint& waypoint) const {
        Segment s(waypoint);
        return with_segment_at_end(s);
    }

    /** Increase of length as result of inserting the given segment at the given position */
    double insertion_length_cost(size_t insert_loc, const Segment segment) const {
        if(traj.size() == 0)
            return segment.length;
        else if(insert_loc == 0)
            return segment.length + conf->uav.travel_distance(segment.end, traj[insert_loc].start);
        else if(insert_loc == traj.size())
            return conf->uav.travel_distance(traj[insert_loc - 1].end, segment.start) + segment.length;
        else {
            return conf->uav.travel_distance(traj[insert_loc - 1].end, segment.start)
                   + segment.length
                   + conf->uav.travel_distance(segment.end, traj[insert_loc].start)
                   - conf->uav.travel_distance(traj[insert_loc - 1].end, traj[insert_loc].start);
        }
    }
    double insertion_duration_cost(size_t insert_loc, const Segment segment) const {
        return insertion_length_cost(insert_loc, segment) / conf->uav.max_air_speed;
    }

    /** Decrease of length as result of removing the segment at the given position */
    double removal_length_gain(size_t index) const {
        const Segment segment = traj[index];
        if(index == 0)
            return segment.length + conf->uav.travel_distance(segment.end, traj[index].start);
        else if(index == traj.size()-1)
            return conf->uav.travel_distance(traj[index - 1].end, segment.start) + segment.length;
        else {
            return conf->uav.travel_distance(traj[index - 1].end, segment.start)
                   + segment.length
                   + conf->uav.travel_distance(segment.end, traj[index].start)
                   - conf->uav.travel_distance(traj[index - 1].end, traj[index].start);
        }
    }
    double removal_duration_gain(size_t index) const {
        return removal_length_gain(index) / conf->uav.max_air_speed;
    }

    /** Computes the cost of replacing the N segments at [index, index+N] with the N segments given in parameter. */
    double replacement_length_cost(size_t index, const std::vector<Segment>& segments) const {
        const unsigned long end_index = index + segments.size() - 1;
        ASSERT(index >= 0 && end_index < traj.size())
        double cost =
                segments_length(segments, 0, segments.size())
                - segments_length(traj, index, segments.size());
        if(index > 0)
            cost = cost
                   + conf->uav.travel_distance(traj[index-1].end, segments[0].start)
                   - conf->uav.travel_distance(traj[index-1].end, traj[index].start);
        if(end_index+1 < traj.size())
            cost = cost
                   + conf->uav.travel_distance(segments[segments.size()-1].end, traj[end_index+1].start)
                   - conf->uav.travel_distance(traj[end_index].end, traj[end_index+1].start);

        return cost;
    }
    double replacement_duration_cost(size_t index, const std::vector<Segment>& segments) const {
        return replacement_length_cost(index, segments) / conf->uav.max_air_speed;
    }

    /** Returns a new trajectory with an additional segment at the given index */
    Trajectory with_additional_segment(size_t index, const Segment& seg) const {
        Trajectory newTraj(*this);
        newTraj.insert_segment(seg, index);
        return newTraj;
    }

    /** Adds segment to the end of the trajectory */
    void append_segment(const Segment& seg) {
        insert_segment(seg, traj.size());
    }

    /** Inserts the given segment at the given index */
    void insert_segment(const Segment& seg, size_t at_index) {
        const double start = at_index == 0 ?
                          conf->start_time :
                          end_time(at_index-1) + conf->uav.travel_time(traj[at_index-1].end, seg.start);

        const double added_delay = insertion_duration_cost(at_index, seg);
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
        traj.erase(traj.begin()+at_index);
        start_times.erase(start_times.begin()+at_index);
        for(size_t i=at_index; i<start_times.size(); i++) {
            start_times[i] -= gained_delay;
        }
        check_validity();
    }

    /** In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
    Trajectory with_replaced_section(size_t index, const std::vector<Segment>& segments) const {
        Trajectory newTraj(*this);
        newTraj.replace_section(index, segments);
        return newTraj;
    }
    void replace_section(size_t index, const std::vector<Segment>& segments) {
        ASSERT(index >= 0 && index+segments.size()-1 < traj.size())
        for(size_t i=0; i<segments.size(); i++) {
            erase_segment(index);
        }
        for(size_t i=0; i<segments.size(); i++) {
            insert_segment(segments[i], index+1);
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
    double length() const {
        return segments_length(traj, 0, traj.size());
    }

    /** Functions that asserts invariants of a trajectory.
     * This is for debugging purpose only and the function content should be commented out. */
    void check_validity() const {
//        ASSERT(fabs(length() / conf->uav.max_air_speed - (end_time() - start_time())) < 0.001)
        ASSERT(matches_configuration())
    }

    /** Converts a vector of waypoints to a vector of segments. */
    static std::vector<Segment> segments_from_waypoints(std::vector<Waypoint> waypoints) {
        std::vector<Segment> segments;
        std::transform(waypoints.begin(), waypoints.end(), segments.begin(),
                       [](const Waypoint& wp) { return Segment(wp); });
        return segments;
    }

    /** Computes the cost of a sub-trajectory composed of the given segments. */
    double segments_length(const std::vector<Segment>& segments, size_t start, size_t length) const {
        ASSERT(start+length <= segments.size());
        double cost = 0.;
        auto end_it = segments.begin() + start + length;
        for(auto it=segments.begin()+start; it!=end_it; it++) {
            cost += it->length;
            if((it+1) != end_it)
                cost += conf->uav.travel_distance(it->end, (it+1)->start);
        }
        return cost;
    }

    /** Returns true if this trajectory is valid wrt the given configuration (has the right start/end points and is not to long. */
    bool matches_configuration() const {
        const bool start_valid = !conf->start_position ||
                (traj.size() >= 1 && traj[0].start == *conf->start_position &&
                        traj[0].end == *conf->start_position && traj[0].length == 0);
        const size_t last_index = traj.size()-1;
        const bool end_valid = !conf->end_position ||
                (traj.size() >= 1 && traj[last_index].start == *conf->end_position &&
                        traj[last_index].end == *conf->end_position && traj[last_index].length == 0);
        const bool duration_valid = duration() <= conf->max_flight_time;

        return start_valid && end_valid && duration_valid;
    }
};

#endif //PLANNING_CPP_TRAJECTORY_H
