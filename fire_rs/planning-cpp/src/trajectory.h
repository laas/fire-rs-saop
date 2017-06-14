#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#import <string>
#include <sstream>
#include <algorithm>
#include <cassert>
#include "ext/dubins.h"
#include "waypoint.h"
#include "uav.h"
#include "debug.h"


struct Segment {
    Waypoint start;
    Waypoint end;
    double length;

    constexpr Segment(const Waypoint& wp1) : start(wp1), end(wp1), length(0) {}
    constexpr Segment(const Waypoint& wp1, const Waypoint& wp2)
            : start(wp1), end(wp2), length(sqrt(pow(wp1.x-wp2.x, 2) + pow(wp1.y-wp2.y, 2))) {}
    constexpr Segment(const Waypoint& wp1, const double length)
            : start(wp1), end(Waypoint(wp1.x +cos(wp1.dir)*length, wp1.y + sin(wp1.dir)*length, wp1.dir)), length(length) {}
};


class Trajectory {
public:
    UAV uav;
    std::vector<Segment> traj;

    Trajectory(const Trajectory& trajectory) = default;

private:
    /** length is lazily computed and should be accessed through the length() method.
     * mutable keyword is there to allow defining const methods */
    mutable double _length = -1;

public:
    double length() const {
        if(_length < 0) {
            _length = segments_cost(traj, 0, traj.size());
        }
        // expensive check that incremental length computation is correct
        // assert(fabs(_length - segments_cost(traj, 0, traj.size())) < 0.0001);
        return _length;
    }

    double duration() const { return length() / uav.max_air_speed; }

    /** Empty trajectory constructor */
    Trajectory(const UAV& uav) : uav(uav), traj(std::vector<Segment>()), _length(0){}

    Trajectory(const UAV& uav, std::vector<Segment> traj) : uav(uav), traj(std::vector<Segment>(traj)) {}

    Trajectory(const UAV& uav, std::vector<Segment> traj, double length) : uav(uav), traj(std::vector<Segment>(traj)), _length(length) {}

    Trajectory(const UAV& uav, std::vector<Waypoint> waypoints) : uav(uav), traj(Trajectory::segments_from_waypoints(waypoints)) {}

    /** Accesses the index-th segement of the trajectory */
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
                auto local_sampled = uav.path_sampling(waypoints[i], waypoints[i+1], step_size);
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
            return segment.length + uav.travel_distance(segment.end, traj[insert_loc].start);
        else if(insert_loc == traj.size())
            return uav.travel_distance(traj[insert_loc - 1].end, segment.start) + segment.length;
        else {
            return uav.travel_distance(traj[insert_loc - 1].end, segment.start)
                   + segment.length
                   + uav.travel_distance(segment.end, traj[insert_loc].start)
                   - uav.travel_distance(traj[insert_loc - 1].end, traj[insert_loc].start);
        }
    }
    double insertion_duration_cost(size_t insert_loc, const Segment segment) const {
        return insertion_length_cost(insert_loc, segment) / uav.max_air_speed;
    }

    /** Decrease of length as result of removing the segment at the given position */
    double removal_length_gain(size_t index) const {
        const Segment segment = traj[index];
        if(index == 0)
            return segment.length + uav.travel_distance(segment.end, traj[index].start);
        else if(index == traj.size()-1)
            return uav.travel_distance(traj[index - 1].end, segment.start) + segment.length;
        else {
            return uav.travel_distance(traj[index - 1].end, segment.start)
                   + segment.length
                   + uav.travel_distance(segment.end, traj[index].start)
                   - uav.travel_distance(traj[index - 1].end, traj[index].start);
        }
    }
    double removal_duration_gain(size_t index) const {
        return removal_length_gain(index) / uav.max_air_speed;
    }

    /** Computes the cost of replacing the N segments at [index, index+N] with the N segments given in parameter. */
    double replacement_length_cost(size_t index, const std::vector<Segment>& segments) const {
        const unsigned long end_index = index + segments.size() - 1;
        ASSERT(index >= 0 && end_index < traj.size())
        double cost =
                segments_cost(segments, 0, segments.size())
                - segments_cost(traj, index, segments.size());
        if(index > 0)
            cost = cost
                   + uav.travel_distance(traj[index-1].end, segments[0].start)
                   - uav.travel_distance(traj[index-1].end, traj[index].start);
        if(end_index+1 < traj.size())
            cost = cost
                   + uav.travel_distance(segments[segments.size()-1].end, traj[end_index+1].start)
                   - uav.travel_distance(traj[end_index].end, traj[end_index+1].start);

        return cost;
    }
    double replacement_duration_cost(size_t index, const std::vector<Segment>& segments) const {
        return replacement_length_cost(index, segments) / uav.max_air_speed;
    }

    /** Returns a new trajectory with an additional segment at the given index */
    Trajectory with_additional_segment(size_t index, const Segment& seg) const {
        std::vector<Segment> newTraj(traj);
        newTraj.insert(newTraj.begin()+index, seg);
        return Trajectory(uav, newTraj, length() + insertion_length_cost(index, seg));
    }

    /** Returns a new trajectory without the segment at the given index */
    Trajectory without_segment(size_t index) const {
        std::vector<Segment> newTraj(traj);
        newTraj.erase(newTraj.begin()+index);
        return Trajectory(uav, newTraj, length() - removal_length_gain(index));
    }

    /** In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
    Trajectory with_replaced_section(size_t index, const std::vector<Segment>& segments) const {
        const unsigned long end_index = index + segments.size() - 1;
        ASSERT(index >= 0 && end_index < traj.size())
        std::vector<Segment> newTraj(traj);
        for(unsigned long i=0; i<segments.size(); i++) {
           newTraj[index+i] = segments[i];
        }
        Trajectory t(uav, newTraj, length() + replacement_length_cost(index, segments));
        t.length();
        return t;
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
    /** Converts a vector of waypoints to a vector of segments. */
    static std::vector<Segment> segments_from_waypoints(std::vector<Waypoint> waypoints) {
        std::vector<Segment> segments;
        std::transform(waypoints.begin(), waypoints.end(), segments.begin(),
                       [](const Waypoint& wp) { return Segment(wp); });
        return segments;
    }

    /** Computes the cost of a sub-trajectory composed of the given segments. */
    double segments_cost(const std::vector<Segment>& segments, unsigned long start, unsigned long length) const {
        double cost = 0.;
        auto end_it = segments.begin() + start + length;
        for(auto it=segments.begin()+start; it!=end_it; it++) {
            cost += it->length;
            if((it+1) != end_it)
                cost += uav.travel_distance(it->end, (it+1)->start);
        }
        return cost;
    }
};

#endif //PLANNING_CPP_TRAJECTORY_H
