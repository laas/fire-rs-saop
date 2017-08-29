#ifndef PLANNING_CPP_SMOOTHING_H
#define PLANNING_CPP_SMOOTHING_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"
#include "../utils.h"
#include "moves.h"

struct TrajectorySmoothingNeighborhood final : public Neighborhood {

    /** Maximum number of changes to try before returning. */
    const size_t max_trials;

    explicit TrajectorySmoothingNeighborhood(size_t max_trials = 10) : max_trials(max_trials) {}

    opt<PLocalMove> get_move(PPlan plan) override {

        double plan_base_utility = plan->utility();

        size_t trials = 0;
        while(++trials < max_trials) {
            // pick a random trajectory in plan.
            const size_t traj_id = rand(0, plan->trajectories.size());
            const Trajectory& traj = plan->trajectories[traj_id];

            if (traj.size() <= 2) {
                continue;
            }

            // pick a random segment in the trajectory
            const opt<size_t> opt_seg_id = traj.random_modifiable_id();

            if (!opt_seg_id) {
                continue;
            }
            
            const size_t seg_id = *opt_seg_id;
            const Segment seg = traj[seg_id];

            const bool can_join_backwards = seg_id - 1 >= traj.first_modifiable();
            const bool can_join_forward = seg_id + 1 <= traj.last_modifiable();

            double backwards_duration = std::numeric_limits<double>::infinity();
            double forward_duration = std::numeric_limits<double>::infinity();

            opt<Segment> prev;
            opt<Segment> next;
            std::vector<Segment> prev_replacement = {};
            std::vector<Segment> next_replacement = {};

            PLocalMove move;

            if (can_join_backwards) {
                prev = traj[seg_id - 1];
                Segment prev_replacement_seg = Segment(prev->start.as_point(), seg.end.as_point());
                if (prev_replacement_seg.length > 500) {
                    continue;
                }
                prev_replacement.push_back(prev_replacement_seg);

                backwards_duration = traj.replacement_duration_cost(seg_id - 1, 2, prev_replacement);

                move = make_shared<SegmentReplacement>(SegmentReplacement(plan, traj_id, seg_id - 1, 2, prev_replacement));
                double move_utility = move->utility();
                double move_duration = move->duration();
                if(move->is_valid() && (move_utility < plan->utility() || move_duration < plan->duration())) {
                    return move;
                }
            }

            if (can_join_forward) {
                next = traj[seg_id + 1];
                Segment next_replacement_seg = Segment(seg.start.as_point(), next->end.as_point());
                if (next_replacement_seg.length > 500) {
                    continue;
                }
                next_replacement.push_back(next_replacement_seg);

                forward_duration = traj.replacement_duration_cost(seg_id, 2, next_replacement);

                move = make_shared<SegmentReplacement>(SegmentReplacement(plan, traj_id, seg_id, 2, next_replacement));
                double move_utility = move->utility();
                double move_duration = move->duration();
                if(move->is_valid() && (move_utility < plan->utility() || move_duration < plan->duration())) {
                    return move;
                }
            }

            return {};

//            if (backwards_duration < 0. && backwards_duration < forward_duration) {
//                PLocalMove move = make_shared<SegmentReplacement>(plan, traj_id, seg_id, 1,prev_replacement);
//                if(move->is_valid() && move->utility() <= plan_base_utility) {
//                    return move;
//                }
//            } else { // backwards_duration is > forward_duration
//                if (forward_duration < 0.) {
//                    PLocalMove move = make_shared<SegmentReplacement>(plan, traj_id, seg_id, 1, next_replacement);
//                    if(move->is_valid() && move->utility() <= plan_base_utility) {
//                        return move;
//                    }
//                }
//            }

        }
        return {}; // back and forward are infinite for every trial
    }
};

#endif //PLANNING_CPP_SMOOTHING_H
