#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"
#include "moves.h"

/** Neighborhood that assigns a random orientation to a random segment. */
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

/** Neighborhood that assign to random segment, the direction from its predecessor to its successor. */
struct MeanOrientationChangeNbhd final : public Neighborhood {

    opt<PLocalMove> get_move(PPlan plan) {
        ASSERT(!plan->trajectories.empty())
        const size_t rand_traj = rand(0, plan->trajectories.size());
        if(plan->trajectories[rand_traj].empty()) {
            return {};
        } else {
            const Trajectory& traj = plan->trajectories[rand_traj];
            const size_t rand_seg = rand(0, plan->trajectories[rand_traj].size());
            if(rand_seg == 0 || rand_seg == traj.size()-1)
                return {};

            auto dx = traj[rand_seg+1].start.x - traj[rand_seg-1].end.x;
            auto dy = traj[rand_seg+1].start.y - traj[rand_seg-1].end.y;
            auto mean_angle = atan2(dy, dx);
            SegmentRotation move(plan, rand_traj, rand_seg, mean_angle);
            if(move.applicable()) {
                return opt<PLocalMove>(make_shared<SegmentRotation>(move));
            } else {
                return {};
            }
        }
    }
};

/** Combination of all neighborhoods intended to smooth dubins trajectories. */
struct DubinsOptNeighborhood final : public CombinedNeighborhood {

    DubinsOptNeighborhood() :
            CombinedNeighborhood(vector<shared_ptr<Neighborhood>> {
                    (shared_ptr<Neighborhood>) make_shared<MeanOrientationChangeNbhd>(),
                    (shared_ptr<Neighborhood>) make_shared<RandomOrientationChangeNbhd>()
            }) {}

};

#endif //PLANNING_CPP_DUBINS_OPTIMIZATION_H
