#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#import <string>
#include <sstream>
#include <algorithm>
#include <cassert>
#include "dubins.h"

struct Waypoint {
    // order and length is important for compatibility with dubins.cpp, allowing to simply cast a Waypoint* to a double*
    double x;
    double y;
    double dir;

    constexpr Waypoint(const Waypoint &wp) : x(wp.x), y(wp.y), dir(wp.dir) {}
    constexpr Waypoint(const double x, const double y, const double dir) : x(x), y(y), dir(dir) {};

    std::string to_string() const {
        std::stringstream repr;
        repr << "(" << x<<", "<<y<<", "<<dir<<")";
        return repr.str();
    }
};

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

struct UAV {
    const double rho;
    const double speed;

    constexpr UAV(const double rho, const double speed) : rho(rho), speed(speed) {};

    /** Returns the Dubins travel distance between the two waypoints. */
    double travel_distance(const Waypoint &origin, const Waypoint &target) const {
        DubinsPath path = dubins_path(origin, target);
        return dubins_path_length(&path);
    }

    /** Returns the travel time between the two waypoints. */
    double travel_time(const Waypoint& origin, const Waypoint &target) const {
        return travel_distance(origin, target) / speed;
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

private:
    DubinsPath dubins_path(const Waypoint &origin, const Waypoint &target) const {
        DubinsPath path;
        // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
        int ret = dubins_init((double*) &origin, (double*) &target, rho, &path);
        assert(ret == 0);
        return path;
    }
};


class Trajectory {
public:
    const UAV uav;
    const std::vector<Segment> traj;

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

    double duration() const { return length() / uav.speed; }

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
    double insertion_cost(unsigned long index, const Segment segment) const {
        if(traj.size() == 0)
            return segment.length;
        else if(index == 0)
            return segment.length + uav.travel_distance(segment.end, traj[index].start);
        else if(index == traj.size())
            return uav.travel_distance(traj[index - 1].end, segment.start) + segment.length;
        else {
            return uav.travel_distance(traj[index - 1].end, segment.start)
                   + segment.length
                   + uav.travel_distance(segment.end, traj[index].start)
                   - uav.travel_distance(traj[index - 1].end, traj[index].start);
        }
    }

    /** Decrease of length as result of removing the segment at the given position */
    double removal_gain(unsigned long index) const {
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

    /** Computes the cost of replacing the N segments at [index, index+N] with the N segments given in parameter. */
    double replacement_cost(unsigned long index, const std::vector<Segment>& segments) const {
        const unsigned long end_index = index + segments.size() - 1;
        assert(index >= 0 && end_index < traj.size());
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

    /** Returns a new trajectory with an additional segment at the given index */
    Trajectory with_additional_segment(unsigned long index, const Segment& seg) const {
        std::vector<Segment> newTraj(traj);
        newTraj.insert(newTraj.begin()+index, seg);
        return Trajectory(uav, newTraj, length() + insertion_cost(index, seg));
    }

    /** Returns a new trajectory without the segment at the given index */
    Trajectory without_segment(unsigned long index) const {
        std::vector<Segment> newTraj(traj);
        newTraj.erase(newTraj.begin()+index);
        return Trajectory(uav, newTraj, length() - removal_gain(index));
    }

    /** In a new trajectory, replaces the N segments at [index, index+N] with the N segments given in parameter. */
    Trajectory with_replaced_section(unsigned long index, const std::vector<Segment>& segments) const {
        std::vector<Segment> newTraj(traj);
        for(unsigned long i=0; i<segments.size(); i++) {
           newTraj[index+i] = segments[i];
        }
        Trajectory t(uav, newTraj, length() + replacement_cost(index, segments));
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
