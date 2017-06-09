#ifndef PLANNING_CPP_PLANNING_H
#define PLANNING_CPP_PLANNING_H

#include <vector>
#include "trajectory.h"
#include "visibility.h"
#import "optional.h"

using namespace std;

struct Plan;
typedef shared_ptr<Plan> PPlan;

struct Plan {
    vector<Trajectory> trajectories;
    Visibility visibility;

    double cost = -1;
    double length = -1;

    Plan(const Plan& plan) = default;

    Plan(vector<tuple<UAV&,Waypoint,Waypoint>> uav_start_end, Visibility visibility) :
            visibility(visibility)
    {
        for(auto it=uav_start_end.begin(); it!=uav_start_end.end(); it++) {
            auto traj = Trajectory(std::get<0>(*it), vector<Waypoint> { std::get<1>(*it), std::get<2>(*it) });
            trajectories.push_back(traj);
        }
        compute_duration_and_cost();
    }

    Plan(vector<UAV> uavs, Visibility visibility) :
            visibility(visibility)
    {
        for(auto it=uavs.begin(); it!=uavs.end(); it++) {
            auto traj = Trajectory(*it, vector<Waypoint> { });
            trajectories.push_back(traj);
        }
        compute_duration_and_cost();
    }

    void compute_duration_and_cost() {
        length = 0;
        for(auto it=trajectories.begin(); it!=trajectories.end(); it++)
            length += it->length();

        cost = visibility.cost();
    }

    void add_segment(size_t traj_id, Segment seg, size_t insert_loc) {
        assert(traj_id < trajectories.size());
        assert(insert_loc <= trajectories[traj_id].traj.size());
        trajectories[traj_id] = trajectories[traj_id].with_additional_segment(insert_loc, seg);
        visibility.add_segment(trajectories[traj_id].uav, seg);
        compute_duration_and_cost();
    }

};


class LocalMove {
protected:
    double _cost = -1;
    double length = -1;

    static constexpr double INF = 9999999999999999;

public:
    PPlan base_plan;
    LocalMove(PPlan base) : base_plan(base) {}
    double cost() const { return _cost; };
    double duration() const { return length; };
    virtual PPlan apply() = 0;
};

struct Insert : LocalMove {
    const size_t traj_id;
    const Segment seg;
    const size_t insert_loc;

    Insert(PPlan base, size_t traj_id, Segment seg, size_t insert_loc)
            : LocalMove(base),
              traj_id(traj_id),
              seg(seg),
              insert_loc(insert_loc)
    {
        assert(traj_id < base->trajectories.size());
        assert(insert_loc <= base->trajectories[traj_id].traj.size());
        _cost = base->visibility.cost_given_addition(base->trajectories[traj_id].uav, seg);
        length = base->length + base->trajectories[traj_id].insertion_cost(insert_loc, seg);
    }

    PPlan apply() {
        auto p = make_shared<Plan>(*base_plan);
        p->add_segment(traj_id, seg, insert_loc);
        assert(base_plan->trajectories[traj_id].traj.size() == p->trajectories[traj_id].traj.size() -1);
        return p;
    }

    static experimental::optional<Insert> best_insert(PPlan base, size_t traj_id, Segment seg, double max_cost=INF) {
        assert(traj_id < base->trajectories.size());

        if(max_cost == INF || max_cost >= base->visibility.cost_given_addition(base->trajectories[traj_id].uav, seg)) {
            long best_loc = -1;
            double best_length = INF;
            Trajectory& traj = base->trajectories[traj_id];
            for(size_t i=0; i<=traj.traj.size(); i++) {
                const double length = traj.length() + traj.insertion_cost(i, seg);
                if(best_length > length) {
                    best_length = length;
                    best_loc = i;
                }
            }
            if(best_loc < 0) {
                return {};
            }
            else {
                return Insert(base, traj_id, seg, (size_t) best_loc);
            }
        } else {
            return {};
        }
    }

    static PPlan smart_insert(PPlan base, size_t traj_id, vector<Segment> segments) {
        experimental::optional<PPlan> current = base;
        for(auto it=segments.begin(); it!=segments.end() && current; it++) {
            current = best_insert(*current, traj_id, *it)->apply();
        }
        return *current;
    }
};




#endif //PLANNING_CPP_PLANNING_H
