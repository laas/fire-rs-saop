#ifndef PROJECT_TRAJECTORIES_H
#define PROJECT_TRAJECTORIES_H

#include <ostream>
#include "trajectory.h"

using namespace std;

struct Trajectories {

    vector<Trajectory> trajectories;

    explicit Trajectories(const vector<Trajectory> &trajectories) : trajectories(trajectories) {}

    explicit Trajectories(vector<TrajectoryConfig> traj_confs)
    {
        for (auto& conf : traj_confs) {
            auto traj = Trajectory(conf);
            trajectories.push_back(traj);
        }
    }

    friend ostream &operator<<(ostream &os, const Trajectories &trajectories) {
        os << "#trajectories: " << trajectories.trajectories.size();
        return os;
    }

    /** A plan is valid iff all trajectories are valid (match their configuration. */
    bool is_valid() const {
        for(auto traj : trajectories)
            if(!traj.has_valid_flight_time())
                return false;
        return true;
    }

    /** Sum of all trajectory durations. */
    double duration() const {
        double duration = 0;
        for(auto& traj : trajectories)
            duration += traj.duration();
        return duration;
    }

    size_t num_segments() const {
        size_t total = 0;
        for(auto traj : trajectories)
            total += traj.traj.size();
        return total;
    }

    /** Returns the UAV performing the given trajectory */
    UAV uav(size_t traj_id) const {
        ASSERT(traj_id < trajectories.size())
        return trajectories[traj_id].conf.uav;
    }
};

#endif //PROJECT_TRAJECTORIES_H
