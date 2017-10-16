//
// Created by saop on 10/16/17.
//

#include "updates.h"

PReversiblePlanUpdate SequencedUpdates::apply(Trajectories& p) {
    PReversiblePlanUpdate first_reverse = first->apply(p);
    PReversiblePlanUpdate second_reverse = second->apply(p);
    return make_shared<SequencedUpdates>(second_reverse, first_reverse);
}

PReversiblePlanUpdate InsertSegment::apply(Trajectories &p) {
    ASSERT(traj_id < p.trajectories.size());
    ASSERT(insert_loc <= p.trajectories[traj_id].traj.size());
    p.trajectories[traj_id].insert_segment(seg, insert_loc);
    return make_shared<DeleteSegment>(traj_id, insert_loc);
}

PReversiblePlanUpdate DeleteSegment::apply(Trajectories &p) {
    ASSERT(traj_id < p.trajectories.size());
    ASSERT(at_index < p.trajectories[traj_id].traj.size());
    Segment3d seg = p.trajectories[traj_id][at_index];
    p.trajectories[traj_id].erase_segment(at_index);
    return make_shared<InsertSegment>(traj_id, seg, at_index);
}
