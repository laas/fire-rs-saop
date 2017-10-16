#ifndef CORE_STRUCTURES_UPDATES_H
#define CORE_STRUCTURES_UPDATES_H

#include "../structures/trajectories.h"

using namespace std;

struct ReversiblePlanUpdate;
typedef shared_ptr<ReversiblePlanUpdate> PReversiblePlanUpdate;

struct DeleteSegment;

struct ReversiblePlanUpdate {

    virtual PReversiblePlanUpdate apply(Trajectories& p) = 0;
};

struct NoOp final : public ReversiblePlanUpdate {
    PReversiblePlanUpdate apply(Trajectories &p) override {
        return make_shared<NoOp>();
    }

};

struct SequencedUpdates final : public ReversiblePlanUpdate {
    PReversiblePlanUpdate first;
    PReversiblePlanUpdate second;

    SequencedUpdates(const PReversiblePlanUpdate first, const PReversiblePlanUpdate second) : first(first),
                                                                                              second(second) {}

    PReversiblePlanUpdate apply(Trajectories &p) override;
};

struct InsertSegment final : public ReversiblePlanUpdate {
    /** Index of the trajectory in which to perform the insertion. */
    const size_t traj_id;

    /** Segment to insert. */
    const Segment3d seg;

    /** Place in the trajectory where this segment should be inserted. */
    const size_t insert_loc;

    InsertSegment(size_t traj_id, const Segment3d &seg, size_t insert_loc) : traj_id(traj_id), seg(seg),
                                                                             insert_loc(insert_loc) {}

    PReversiblePlanUpdate apply(Trajectories &p) override;
};

struct DeleteSegment final : public ReversiblePlanUpdate {

    /** Index of the trajectory in which to perform the insertion. */
    const size_t traj_id;

    /** Place in the trajectory where this segment should be inserted. */
    const size_t at_index;

    DeleteSegment(const size_t traj_id, const size_t at_index) : traj_id(traj_id), at_index(at_index) {}

    PReversiblePlanUpdate apply(Trajectories &p) override;
};



#endif //PROJECT_TRAJECTORIES_H
