#ifndef PLANNING_CPP_CONTOUR_FOLLOWING_H
#define PLANNING_CPP_CONTOUR_FOLLOWING_H

#include "../vns_interface.h"
#include "moves.h"


struct ForcedSegmentSwap : public SegmentSwap {

    ForcedSegmentSwap(const PPlan &base, size_t traj_id, size_t segment_index, const Segment& replacement)
            : SegmentSwap(base,
                          traj_id,
                          segment_index,
                          replacement) {
        _compulsory = true;
    }

};

struct ForcedRemoval : public Remove {
    ForcedRemoval(const PPlan &base, size_t traj_id, size_t rm_id) : Remove(base, traj_id, rm_id) {
        _compulsory = true;
    }
};

struct ProjectOnFrontNbhd final : public Neighborhood {

    ProjectOnFrontNbhd() {
        // this one is deterministic and sequential
        max_neighbors = 1;
    }

    opt<PLocalMove> get_move(PPlan plan) {
        ASSERT(!plan->trajectories.empty())
        for(size_t traj_id=0; traj_id<plan->trajectories.size(); traj_id++) {
            for(size_t seg_id=0; seg_id<plan->trajectories[traj_id].size(); seg_id++) {
                const Segment& initial_segment = plan->trajectories[traj_id][seg_id];
                const double segment_start = plan->trajectories[traj_id].start_time(seg_id);
                auto projected = plan->visibility.fire.project_on_firefront(initial_segment, plan->uav(traj_id), segment_start);
                if(projected) {
                    if(*projected != initial_segment) {
                        auto move = make_shared<ForcedSegmentSwap>(plan, traj_id, seg_id, *projected);
                        if(move->applicable())
                            return opt<PLocalMove>(move);
                        else {
                            auto rm_move = make_shared<ForcedRemoval>(plan, traj_id, seg_id);
                            return opt<PLocalMove>(rm_move);
                        }
                    }
                } else {
                    // could not project segment on firefront, remove it
                    auto rm_move = make_shared<ForcedRemoval>(plan, traj_id, seg_id);
                    return opt<PLocalMove>(rm_move);
                }
            }
        }
        // all segments are on the firefront, nothing to do
        return {};
    }
};

#endif //PLANNING_CPP_CONTOUR_FOLLOWING_H
