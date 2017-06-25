#ifndef PLANNING_CPP_PLAN_H
#define PLANNING_CPP_PLAN_H


#include "trajectory.h"
#include "visibility.h"

struct Plan;
typedef shared_ptr<Plan> PPlan;

struct Plan {
    vector<Trajectory> trajectories;
    Visibility visibility;

    Plan(const Plan& plan) = default;

    Plan(vector<TrajectoryConfig> traj_confs, Visibility visibility) :
            visibility(visibility)
    {
        for(auto conf=traj_confs.begin(); conf!=traj_confs.end(); conf++) {
            auto traj = Trajectory(*conf);
            trajectories.push_back(traj);
        }
    }

    double duration() const {
        double duration = 0;
        for(auto it=trajectories.begin(); it!=trajectories.end(); it++) {
            duration += it->duration();
        }
        return duration;
    }

    double cost() const {
        return visibility.cost();
    }

    /** Returns the UAV performing the given trajectory */
    UAV uav(size_t traj_id) const {
        ASSERT(traj_id < trajectories.size())
        return trajectories[traj_id].conf->uav;
    }

    void insert_segment(size_t traj_id, const Segment& seg, size_t insert_loc) {
        ASSERT(traj_id < trajectories.size());
        ASSERT(insert_loc <= trajectories[traj_id].traj.size());
        trajectories[traj_id].insert_segment(seg, insert_loc);
        visibility.add_segment(trajectories[traj_id].conf->uav, seg);
    }

    void erase_segment(size_t traj_id, size_t at_index) {
        ASSERT(traj_id < trajectories.size());
        ASSERT(at_index < trajectories[traj_id].traj.size());
        Segment deleted = trajectories[traj_id][at_index];
        trajectories[traj_id].erase_segment(at_index);
        visibility.remove_segment(trajectories[traj_id].conf->uav, deleted);
    }

    void replace_segment(size_t traj_id, size_t at_index, const Segment& by_segment) {
        ASSERT(traj_id < trajectories.size());
        ASSERT(at_index <= trajectories[traj_id].traj.size());
        erase_segment(traj_id, at_index);
        insert_segment(traj_id, by_segment, at_index);

    }
};



#endif //PLANNING_CPP_PLAN_H
