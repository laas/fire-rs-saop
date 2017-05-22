#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#import <string>
#include <sstream>
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
        repr << "[" << x<<", "<<y<<", "<<dir<<"]";
        return repr.str();
    }
};

struct Segment {
    Waypoint start;
    Waypoint end;
    double length;

    constexpr Segment(const Waypoint wp1) : start(wp1), end(wp1), length(0) {}
    constexpr Segment(const Waypoint wp1, const Waypoint wp2)
            : start(wp1), end(wp2), length(sqrt(pow(wp1.x-wp2.x, 2) + pow(wp1.y-wp2.y, 2))) {}
    constexpr Segment(const Waypoint wp1, const double length)
            : start(wp1), end(Waypoint(wp1.x +cos(wp1.dir)*length, wp1.y + sin(wp1.dir)*length, wp1.dir)), length(length) {}
};

struct UAV {
    const double rho;
    const double speed;

    constexpr UAV(const double rho, const double speed) : rho(rho), speed(speed) {};

    double travel_length(const Waypoint& origin, const Waypoint &target) const {
        DubinsPath path;
        // ugly hack to be compatible with dubins implementation that expects a double[3] in place of each Waypoint
        dubins_init((double*) &origin, (double*) &target, rho, &path);
        return dubins_path_length(&path);
    }

    double travel_time(const Waypoint& origin, const Waypoint &target) const {
        return travel_length(origin, target) / speed;
    }
};


class Trajectory {
    const UAV& uav;
    const std::vector<Segment> traj;

private:
    double _length = -1;

public:
    double length() {
        if(_length < 0) {
            // length not stored, compute it
            _length = 0;
            auto sz = traj.size();
            if(sz == 0) {
                _length = 0;
            } else if(sz == 1) {
                _length = traj[0].length;
            } else {
                auto prev = traj[0];
                _length = prev.length;
                for (unsigned i = 1; i < sz; i++) {
                    _length += uav.travel_length(prev.end, traj[i].start);
                    _length += traj[i].length;
                }
            }
        }

        return _length;
    }

    double duration() { return length() / uav.speed; }

    /** Empty trajectory constructor */
    Trajectory(const UAV& uav) : uav(uav), traj(std::vector<Segment>()), _length(0){}

    Trajectory(const UAV& uav, std::vector<Segment> traj) : uav(uav), traj(std::vector<Segment>(traj)) {}

    Trajectory(const UAV& uav, std::vector<Segment> traj, double length) : uav(uav), traj(std::vector<Segment>(traj)), _length(length) {}

    double insertion_cost(unsigned index, Segment segment) {
        if(traj.size() == 0)
            return segment.length;
        else if(index == 0)
            return segment.length + uav.travel_length(segment.end, traj[index].start);
        else if(index == traj.size())
            return uav.travel_length(traj[index-1].end, segment.start) + segment.length;
        else {
            return uav.travel_length(traj[index-1].end, segment.start)
                   + segment.length
                   + uav.travel_length(segment.end, traj[index].start)
                   - uav.travel_length(traj[index-1].end, traj[index].start);
        }
    }

    double removal_gain(unsigned index) {
        const Segment segment = traj[index];
        if(index == 0)
            return segment.length + uav.travel_length(segment.end, traj[index].start);
        else if(index == traj.size()-1)
            return uav.travel_length(traj[index-1].end, segment.start) + segment.length;
        else {
            return uav.travel_length(traj[index-1].end, segment.start)
                   + segment.length
                   + uav.travel_length(segment.end, traj[index].start)
                   - uav.travel_length(traj[index-1].end, traj[index].start);
        }
    }

    Trajectory with_additionnal_segment(unsigned index, const Segment seg) {
        std::vector<Segment> newTraj(traj);
        newTraj.insert(newTraj.begin()+index, seg);
        return Trajectory(uav, newTraj, length() + insertion_cost(index, seg));
    }

    Trajectory without_segment(unsigned index) {
        std::vector<Segment> newTraj(traj);
        newTraj.erase(newTraj.begin()+index);
        return Trajectory(uav, newTraj, length() - removal_gain(index));
    }
};

#endif //PLANNING_CPP_TRAJECTORY_H
