#ifndef PLANNING_CPP_TRAJECTORY_H
#define PLANNING_CPP_TRAJECTORY_H

#import <vector>
#import <math.h>
#include "dubins.h"

struct Waypoint {
    // order and length is important for compatibility with dubins.cpp, allowing to simply cast a Waypoint* to a double*
    double x;
    double y;
    double dir;

    constexpr Waypoint(const Waypoint &wp) : x(wp.x), y(wp.y), dir(wp.dir) {}
    constexpr Waypoint(const double x, const double y, const double dir) : x(x), y(y), dir(dir) {};
};

struct Segment {
    Waypoint wp1;
    Waypoint wp2;
    double length;

    constexpr Segment(const Waypoint wp1) : wp1(wp1), wp2(wp1), length(0) {}
    constexpr Segment(const Waypoint wp1, const Waypoint wp2)
            : wp1(wp1), wp2(wp2), length(sqrt(pow(wp1.x-wp2.x, 2) + pow(wp1.y-wp2.y, 2))) {}
    constexpr Segment(const Waypoint wp1, const double length)
            : wp1(wp1), wp2(Waypoint(wp1.x +cos(wp1.dir)*length, wp1.y + sin(wp1.dir)*length, wp1.dir)), length(length) {}
};


double travel_time(const Waypoint wp1, const Waypoint wp2, const double rho) {
    DubinsPath path;
    dubins_init((double*) &wp1, (double*) &wp2, rho, &path);
    return dubins_path_length(&path);
}

class Trajectory {
    const std::vector<Segment> traj;
    const double rho = 1;

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
                    _length += travel_time(prev.wp2, traj[i].wp1, rho);
                    _length += traj[i].length;
                }
            }
        }

        return _length;
    }
    /** Empty trajectory constructor */
    Trajectory() : traj(std::vector<Segment>()), _length(0) {}

    Trajectory(std::vector<Segment> traj) : traj(std::vector<Segment>(traj)) {}

    Trajectory(std::vector<Segment> traj, double length) : traj(std::vector<Segment>(traj)), _length(length) {}

    double insertion_cost(unsigned index, Segment segment) {
        if(traj.size() == 0)
            return segment.length;
        else if(index == 0)
            return segment.length + travel_time(segment.wp2, traj[index].wp1, rho);
        else if(index == traj.size())
            return travel_time(traj[index-1].wp2, segment.wp1, rho) + segment.length;
        else {
            return travel_time(traj[index-1].wp2, segment.wp1, rho)
                   + segment.length
                   + travel_time(segment.wp2, traj[index].wp1, rho)
                   - travel_time(traj[index-1].wp2, traj[index].wp1, rho);
        }
    }

    double removal_gain(unsigned index) {
        const Segment segment = traj[index];
        if(index == 0)
            return segment.length + travel_time(segment.wp2, traj[index].wp1, rho);
        else if(index == traj.size())
            return travel_time(traj[index-1].wp2, segment.wp1, rho) + segment.length;
        else {
            return travel_time(traj[index-1].wp2, segment.wp1, rho)
                   + segment.length
                   + travel_time(segment.wp2, traj[index].wp1, rho)
                   - travel_time(traj[index-1].wp2, traj[index].wp1, rho);
        }
    }

    Trajectory with_additionnal_segment(unsigned index, const Segment seg) {
        std::vector<Segment> newTraj(traj);
        newTraj.insert(newTraj.begin()+index, seg);
        return Trajectory(newTraj, length() + insertion_cost(index, seg));
    }

    Trajectory without_segment(unsigned index) {
        std::vector<Segment> newTraj(traj);
        newTraj.erase(newTraj.begin()+index);
        return Trajectory(newTraj, length() - removal_gain(index));
    }
};

#endif //PLANNING_CPP_TRAJECTORY_H
