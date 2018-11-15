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

#include <algorithm>
#include <ostream>
#include "trajectory.hpp"

using namespace std;

namespace SAOP {

    struct Trajectories {

        explicit Trajectories(const vector<Trajectory>& trajectories) : trajs(trajectories) {}

        explicit Trajectories(vector<TrajectoryConfig> traj_confs) {
            for (auto& conf : traj_confs) {
                auto traj = Trajectory(conf);
                trajs.push_back(traj);
            }
        }

        friend ostream& operator<<(ostream& os, const Trajectories& trajectories) {
            os << "#trajectories: " << trajectories.trajs.size();
            return os;
        }

        /** A plan is valid iff all trajectories are valid (match their configuration. */
        bool is_valid() const {
            for (auto& traj : trajs)
                if (!traj.has_valid_flight_time())
                    return false;
            return true;
        }

        /** Sum of all trajectory durations. */
        double duration() const {
            double duration = 0;
            for (auto& traj : trajs)
                duration += traj.duration();
            return duration;
        }

        size_t num_segments() const {
            size_t total = 0;
            for (auto& traj : trajs)
                total += traj.size();
            return total;
        }

        /** Returns the UAV performing the given trajectory */
        UAV uav(size_t traj_id) const {
            ASSERT(traj_id < trajs.size());
            return trajs[traj_id].conf().uav;
        }

        /* For every trajectory, make the maneuvers before and including 'man_id' unmodifiable */
        void freeze_before(double time) {
            for (auto& traj : trajs)
                traj.freeze_before(time);
        }

        void freeze_trajectory(std::string traj_name) {
            auto f_traj = std::find_if(trajs.begin(), trajs.end(),
                                       [&traj_name](const Trajectory& a) { return a.name() == traj_name; });
            if (f_traj != trajs.end()) {
                f_traj->freeze();
            }
        }

        /* For every trajectory, make the maneuvers before and including 'man_id' unmodifiable */
        void erase_modifiable_maneuvers() {
            for (auto& traj : trajs)
                traj.erase_all_modifiable_maneuvers();
        }

        size_t size() const { return trajs.size(); }

        bool empty() const { return size() == 0; }

        const Trajectory& operator[](size_t id) const { return trajs[id]; }

        Trajectory& operator[](size_t id) { return trajs[id]; }

//        const std::vector<Trajectory>& trajectories() const { return trajs; }

        std::vector<Trajectory>::iterator begin() {
            return trajs.begin();
        }

        std::vector<Trajectory>::iterator end() {
            return trajs.end();
        }

        std::vector<Trajectory>::const_iterator begin() const {
            return trajs.begin();
        }

        std::vector<Trajectory>::const_iterator end() const {
            return trajs.end();
        }

        /* Get a refrence to the internal Trajectory vector in Trajectories.
         * To be used only for compatibility in the python interface.*/
        static const std::vector<Trajectory>& get_internal_vector(const Trajectories& ts) { return ts.trajs; };

    private:
        vector<Trajectory> trajs;
    };
}
#endif //PROJECT_TRAJECTORIES_H
