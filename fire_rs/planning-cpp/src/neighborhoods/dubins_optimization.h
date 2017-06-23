#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"

Segment rotate_on_visibility_center(const Segment& segment, const UAV& uav, double target_dir) {
    const double visibility_depth = segment.length + uav.view_depth;
    const double vis_center_x = segment.start.x + cos(segment.start.dir) * visibility_depth/2;
    const double vis_center_y = segment.start.y + sin(segment.start.dir) * visibility_depth/2;
    const double new_segment_start_x = vis_center_x - cos(target_dir) * visibility_depth/2;
    const double new_segment_start_y = vis_center_y - sin(target_dir) * visibility_depth/2;

    return Segment(Waypoint(new_segment_start_x, new_segment_start_y, target_dir), segment.length);
}


struct SegmentRotation final : public LocalMove {
    const size_t traj_id;
    const size_t segment_index;
    const Segment newSegment;
    SegmentRotation(const PPlan &base, size_t traj_id, size_t segment_index, double target_dir)
            : LocalMove(base),
              traj_id(traj_id),
              segment_index(segment_index),
              newSegment(rotate_on_visibility_center(base->trajectories[traj_id][segment_index],
                                                     base->trajectories[traj_id].conf->uav,
                                                     target_dir))
    {
        // assume that cost is not going to change meaningfully since the rotation is designed to
        // minimize the impact on visibility window
        _cost = base->cost;
        _duration = base->duration + base->trajectories[traj_id].replacement_duration_cost(segment_index, newSegment);
        ASSERT(_duration >= 0)
    }

    void apply_on(PPlan p) override {
        p->trajectories[traj_id].replace_segment(segment_index, newSegment);
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
