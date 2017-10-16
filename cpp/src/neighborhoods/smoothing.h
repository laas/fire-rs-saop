#ifndef PLANNING_CPP_SMOOTHING_H
#define PLANNING_CPP_SMOOTHING_H

#include "../core/structures/trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"
#include "../utils.h"
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
