#ifndef PLANNING_CPP_INSERTIONS_H
#define PLANNING_CPP_INSERTIONS_H


#include "../planning.h"
#include "moves.h"


/** Neighborhood for generating Insert moves.
 *
 * The neighborhood randomly picks a pending point in the visibility structures and returns
 * the best insertion for this point.
 */
struct OneInsertNbhd : public Neighborhood {
    const double default_segment_length = 30;

    opt<PLocalMove> get_move(PPlan p) override {
        ASSERT(!p->trajectories.empty())
        if(p->visibility.interesting_pending.size() == 0) {
            // no candidates left
            return {};
        }
        /** Select a random point in the pending list */
        const size_t index = rand() % p->visibility.interesting_pending.size();
        const Cell pt = p->visibility.interesting_pending[index];

        /** Pick an angle randomly */
        const double angle = drand(0, 2*M_PI);

        /** Waypoint and segment resulting from the random picks */
        const Waypoint wp = Waypoint(p->visibility.fire.ignitions.x_coords(pt.x), p->visibility.fire.ignitions.y_coords(pt.y), angle);
        const Segment seg = p->uav(0).observation_segment(wp.x, wp.y, angle, default_segment_length);

        opt<Insert> best_insert = {};

        /** Try best insert for each subtrajectory in the plan */
        for(size_t i=0; i< p->trajectories.size(); i++) {
            auto candidate = Insert::best_insert(p, i, seg);
            if(candidate) {
                if(!best_insert || candidate->is_better_than(*best_insert))
                    best_insert = candidate;
            }
        }

        /** Return the best, if any */
        if(best_insert)
            return opt<PLocalMove>(make_shared<Insert>(*best_insert));
        else
            return {};
    }
};



#endif //PLANNING_CPP_INSERTIONS_H
