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

#ifndef PLANNING_CPP_SMOOTHING_H
#define PLANNING_CPP_SMOOTHING_H

#include "../../core/structures/trajectory.h"
#include "../vns_interface.h"
#include "../../utils.h"
#include "moves.h"

struct TrajectorySmoothingNeighborhood final : public Neighborhood {

    std::string name() const override {
        return "trajectory-smoothing";
    }

    /** Maximum number of changes to try before returning. */
    const size_t max_trials;

    explicit TrajectorySmoothingNeighborhood(size_t max_trials = 10) : max_trials(max_trials) {}

    opt<PLocalMove> get_move(PPlan plan) override {

        size_t trials = 0;
        while(++trials < max_trials) {
            // pick a random trajectory in plan.
            const size_t traj_id = rand(0, plan->core.size());
            const Trajectory& traj = plan->core[traj_id];

            if (traj.size() <= 2) {
                continue;
            }

            // pick a random segment in the trajectory
            const opt<size_t> opt_seg_id = traj.get_random_modifiable_id();

            if (!opt_seg_id) {
                continue;
            }
            
            const size_t seg_id = *opt_seg_id;
            const Segment3d seg = traj[seg_id];

            const bool can_join_backwards = seg_id - 1 >= traj.first_modifiable();
            const bool can_join_forward = seg_id + 1 <= traj.last_modifiable();

            opt<Segment3d> prev;
            opt<Segment3d> next;
            std::vector<Segment3d> prev_replacement = {};
            std::vector<Segment3d> next_replacement = {};

            PLocalMove move;

            if (can_join_backwards) {
                prev = traj[seg_id - 1];
                Segment3d prev_replacement_seg = Segment3d(prev->start.as_point().as_2d(), seg.end.as_point().as_2d(), max(prev->start.z, seg.end.z));
                if (prev_replacement_seg.xy_length <= 500) {
                    prev_replacement.push_back(prev_replacement_seg);

                    move = make_shared<SegmentReplacement>(
                            SegmentReplacement(plan, traj_id, seg_id - 1, 2, prev_replacement));
                    double move_utility = move->utility();
                    double move_duration = move->duration();
                    if (move->is_valid() && (move_utility < plan->utility() && move_duration < plan->duration())) {
                        return move;
                    }
                }
            }

            if (can_join_forward) {
                next = traj[seg_id + 1];
                Segment3d next_replacement_seg = Segment3d(seg.start.as_point().as_2d(), next->end.as_point().as_2d(), max(seg.start.z, next->end.z));
                if (next_replacement_seg.xy_length <= 500) {
                    next_replacement.push_back(next_replacement_seg);

                    move = make_shared<SegmentReplacement>(
                            SegmentReplacement(plan, traj_id, seg_id, 2, next_replacement));
                    double move_utility = move->utility();
                    double move_duration = move->duration();
                    if (move->is_valid() && (move_utility < plan->utility() && move_duration < plan->duration())) {
                        return move;
                    }
                }
            }
        }
        return {}; // back and forward are infinite for every trial
    }
};

#endif //PLANNING_CPP_SMOOTHING_H
