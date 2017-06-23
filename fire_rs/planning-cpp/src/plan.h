#ifndef PLANNING_CPP_PLAN_H
#define PLANNING_CPP_PLAN_H


#include "trajectory.h"
#include "visibility.h"

struct Plan;
typedef shared_ptr<Plan> PPlan;

struct Plan {
    vector<Trajectory> trajectories;
    Visibility visibility;

    double cost = -1;
    double duration = -1;

    Plan(const Plan& plan) = default;

    Plan(vector<TrajectoryConfig> traj_confs, Visibility visibility) :
            visibility(visibility)
    {
        for(auto conf=traj_confs.begin(); conf!=traj_confs.end(); conf++) {
            auto traj = Trajectory(*conf);
            trajectories.push_back(traj);
        }
        compute_duration_and_cost();
    }

    void compute_duration_and_cost() {
        duration = 0;
        for(auto it=trajectories.begin(); it!=trajectories.end(); it++) {
            duration += it->duration();
        }

        cost = visibility.cost();
    }

    void add_segment(size_t traj_id, const Segment& seg, size_t insert_loc) {
        ASSERT(traj_id < trajectories.size());
        ASSERT(insert_loc <= trajectories[traj_id].traj.size());
        trajectories[traj_id].insert_segment(seg, insert_loc);
        visibility.add_segment(trajectories[traj_id].conf->uav, seg);
        compute_duration_and_cost();
    }

};



#endif //PLANNING_CPP_PLAN_H
