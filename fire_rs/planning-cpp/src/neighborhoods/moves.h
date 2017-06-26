#ifndef PLANNING_CPP_MOVES_H
#define PLANNING_CPP_MOVES_H


#include "../plan.h"
#include "../vns_interface.h"

/** Local move that insert a segment at given place in the plan. */
struct Insert final : public LocalMove {
    /** Index of the trajectory in which to perform the insertion. */
    size_t traj_id;

    /** Segment to insert. */
    Segment seg;

    /** Place in the trajectory where this segment should be inserted. */
    size_t insert_loc;

    Insert(PPlan base, size_t traj_id, Segment seg, size_t insert_loc)
            : LocalMove(base),
              traj_id(traj_id),
              seg(seg),
              insert_loc(insert_loc)
    {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(insert_loc <= base->trajectories[traj_id].traj.size());
        Plan clone(*base);
        clone.insert_segment(traj_id, seg, insert_loc);

        _cost = clone.cost();
        _duration = clone.duration();
        is_valid = clone.is_valid();
    }

    void apply_on(PPlan p) override {
        p->insert_segment(traj_id, seg, insert_loc);
    }

    /** Generates an insert move that include the segment at the best place in the given trajectory.
     * Currently, this does not checks the trajectory constraints. */
    static opt<Insert> best_insert(PPlan base, size_t traj_id, Segment seg) {
        ASSERT(traj_id < base->trajectories.size());

        long best_loc = -1;
        double best_dur = INF;
        Trajectory& traj = base->trajectories[traj_id];
        for(size_t i=0; i<=traj.traj.size(); i++) {
            const double dur = traj.duration() + traj.insertion_duration_cost(i, seg);
            if(dur <=  traj.conf.max_flight_time) {
                if (best_dur > dur) {
                    best_dur = dur;
                    best_loc = i;
                }
            }
        }
        if(best_loc < 0) {
            return {};
        }
        else {
            return Insert(base, traj_id, seg, (size_t) best_loc);
        }
    }

    /** Tries to insert each of the segments at the best place in the given trajectory of the given plan */
    static PPlan smart_insert(PPlan base, size_t traj_id, vector<Segment> segments) {
        opt<PPlan> current = base;
        for(auto it=segments.begin(); it!=segments.end() && current; it++) {
            current = best_insert(*current, traj_id, *it)->apply_on_new();
        }
        return *current;
    }
};

/** Local move that insert a segment at given place in the plan. */
struct Remove : public LocalMove {
    /** Index of the trajectory in which to perform the removal. */
    size_t traj_id;

    /** Location of the segment to remove within the trajectory */
    size_t rm_id;

    Remove(PPlan base, size_t traj_id, size_t rm_id)
            : LocalMove(base),
              traj_id(traj_id),
              rm_id(rm_id) {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(rm_id <= base->trajectories[traj_id].traj.size());
        Plan clone(*base);
        auto seg = base->trajectories[traj_id][rm_id];
        _cost = clone.cost();
        _duration = clone.duration();
        is_valid = clone.is_valid();
    }

    void apply_on(PPlan p) override {
        p->erase_segment(traj_id, rm_id);
    }
};

struct SegmentSwap : public LocalMove {
    const size_t traj_id;
    const size_t segment_index;
    const Segment newSegment;

    SegmentSwap(const PPlan &base, size_t traj_id, size_t segment_index, const Segment& replacement)
            : LocalMove(base),
              traj_id(traj_id),
              segment_index(segment_index),
              newSegment(replacement)
    {
        // assume that cost is not going to change meaningfully since the rotation is designed to
        // minimize the impact on visibility window
        Plan clone(*base);
        clone.replace_segment(traj_id, segment_index, newSegment);
        _cost = clone.cost();
        _duration = clone.duration();
        is_valid = clone.is_valid();
        ASSERT(_duration >= 0)
    }

    void apply_on(PPlan p) override {
        p->replace_segment(traj_id, segment_index, newSegment);
    }

    bool applicable() const {
        Trajectory& t = base_plan->trajectories[traj_id];
        return t.duration() + t.replacement_duration_cost(segment_index, newSegment) <= t.conf.max_flight_time;
    }
};


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
        Plan clone(*base);
        clone.replace_segment(traj_id, segment_index, newSegment);
        _cost = base->cost();
        _duration = clone.duration();
        is_valid = clone.is_valid();
        ASSERT(_duration >= 0)
    }

    void apply_on(PPlan p) override {
        p->replace_segment(traj_id, segment_index, newSegment);
    }

    bool applicable() const {
        Trajectory& t = base_plan->trajectories[traj_id];
        return t.duration() + t.replacement_duration_cost(segment_index, newSegment) <= t.conf.max_flight_time;
    }
};

#endif //PLANNING_CPP_MOVES_H
