#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"


struct SegmentRotation final : public LocalMove {
    const size_t traj_id;
    const size_t segment_index;
    const Segment newSegment;
    SegmentRotation(const PPlan &base, size_t traj_id, size_t segment_index, double target_dir)
            : LocalMove(base),
              traj_id(traj_id),
              segment_index(segment_index),
              newSegment(base->uav(traj_id).rotate_on_visibility_center(base->trajectories[traj_id][segment_index],
                                                                        target_dir))
    {
        // assume that cost is not going to change meaningfully since the rotation is designed to
        // minimize the impact on visibility window
        _cost = base->cost();
        _duration = base->duration() + base->trajectories[traj_id].replacement_duration_cost(segment_index, newSegment);
        ASSERT(_duration >= 0)
    }

    void apply_on(PPlan p) override {
        p->replace_segment(traj_id, segment_index, newSegment);
    }

    bool applicable() const {
        Trajectory& t = base_plan->trajectories[traj_id];
        return t.duration() + t.replacement_duration_cost(segment_index, newSegment) <= t.conf->max_flight_time;
    }
};

struct RandomOrientationChangeNbhd final : public Neighborhood {

    opt<PLocalMove> get_move(PPlan plan) {
        ASSERT(!plan->trajectories.empty())
        const size_t rand_traj = rand(0, plan->trajectories.size());
        if(plan->trajectories[rand_traj].empty()) {
            return {};
        } else {
            const size_t rand_seg = rand(0, plan->trajectories[rand_traj].size());
            const double rand_dir = drand(0, 2*M_PI);
            SegmentRotation move(plan, rand_traj, rand_seg, rand_dir);
            if(move.applicable()) {
                return opt<PLocalMove>(make_shared<SegmentRotation>(move));
            } else {
                return {};
            }
        }
    }
};


#endif //PLANNING_CPP_DUBINS_OPTIMIZATION_H
