#ifndef CORE_STRUCTURES_UPDATES_H
#define CORE_STRUCTURES_UPDATES_H

#include "../structures/trajectories.h"

using namespace std;

struct ReversibleTrajectoriesUpdate;
typedef shared_ptr<ReversibleTrajectoriesUpdate> PReversibleTrajectoriesUpdate;

struct DeleteSegment;

struct ReversibleTrajectoriesUpdate {

    virtual PReversibleTrajectoriesUpdate apply(Trajectories& p) = 0;
};

struct NoOp final : public ReversibleTrajectoriesUpdate {
    PReversibleTrajectoriesUpdate apply(Trajectories &p) override {
        return make_shared<NoOp>();
    }

};

struct SequencedUpdates final : public ReversibleTrajectoriesUpdate {
    PReversibleTrajectoriesUpdate first;
    PReversibleTrajectoriesUpdate second;

    SequencedUpdates(const PReversibleTrajectoriesUpdate& first, const PReversibleTrajectoriesUpdate& second) : first(first),
                                                                                              second(second) {}

    PReversibleTrajectoriesUpdate apply(Trajectories &p) override;
};

struct InsertSegment final : public ReversibleTrajectoriesUpdate {
    /** Index of the trajectory in which to perform the insertion. */
    const size_t traj_id;

    /** Segment to insert. */
    const Segment3d seg;

    /** Place in the trajectory where this segment should be inserted. */
    const size_t insert_loc;

    InsertSegment(size_t traj_id, const Segment3d &seg, size_t insert_loc) : traj_id(traj_id), seg(seg),
                                                                             insert_loc(insert_loc) {}

    PReversibleTrajectoriesUpdate apply(Trajectories &p) override;
};

struct DeleteSegment final : public ReversibleTrajectoriesUpdate {

    /** Index of the trajectory in which to perform the insertion. */
    const size_t traj_id;

    /** Place in the trajectory where this segment should be inserted. */
    const size_t at_index;

    DeleteSegment(const size_t traj_id, const size_t at_index) : traj_id(traj_id), at_index(at_index) {}

    PReversibleTrajectoriesUpdate apply(Trajectories &p) override;
};



#endif //PROJECT_TRAJECTORIES_H
