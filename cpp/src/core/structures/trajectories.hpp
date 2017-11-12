/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef PROJECT_TRAJECTORIES_H
#define PROJECT_TRAJECTORIES_H

#include <ostream>
#include "trajectory.hpp"

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
        for(auto& traj : trajectories)
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
        for(auto& traj : trajectories)
            total += traj.traj.size();
        return total;
    }

    /** Returns the UAV performing the given trajectory */
    UAV uav(size_t traj_id) const {
        ASSERT(traj_id < trajectories.size())
        return trajectories[traj_id].conf.uav;
    }

    size_t size() const { return trajectories.size(); }
    bool empty() const { return size() == 0; }

    Trajectory& operator[](size_t id) { return trajectories[id]; }
    const Trajectory& operator[](size_t id) const { return trajectories[id]; }
};

#endif //PROJECT_TRAJECTORIES_H
