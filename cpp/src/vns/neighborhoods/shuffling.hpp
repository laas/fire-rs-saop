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

#ifndef PLANNING_CPP_SHUFFLING_H
#define PLANNING_CPP_SHUFFLING_H

#include "../vns_interface.hpp"

namespace SAOP {

    struct PlanPortionRemover : public Shuffler {

        explicit PlanPortionRemover(double min_removal_portion = 0., double max_removal_portion = 1.) :
                min_removal_portion(min_removal_portion), max_removal_portion(max_removal_portion) {}

        const double min_removal_portion;
        const double max_removal_portion;

        void suffle(shared_ptr<Plan> plan) override {
            for (auto& traj : plan->core.trajectories) {
                if (traj.first_modifiable_id() > traj.last_modifiable_id())
                    // trajectory has no modifiable parts, skip
                    continue;

                const size_t num_removable = traj.last_modifiable_id() + 1 - traj.first_modifiable_id();
                const size_t to_remove_lb = (size_t) max(0, (int) floor(min_removal_portion * num_removable));
                const size_t to_remove_ub = (size_t) min((int) num_removable,
                                                         (int) floor(max_removal_portion * num_removable));
                // number of segments to remove
                const size_t to_remove = rand(to_remove_lb, to_remove_ub + 1);

                size_t next_removal = rand(traj.first_modifiable_id(), traj.last_modifiable_id() + 1);
                size_t removed = 0;
                while (removed < to_remove) {
                    if (next_removal > traj.last_modifiable_id())
                        next_removal = traj.first_modifiable_id();
                    traj.erase_segment(next_removal);
                    removed++;
                }
            }
            plan->post_process();
        }

    };
}
#endif //PLANNING_CPP_SHUFFLING_H
