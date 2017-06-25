#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"
#include "moves.h"


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
