#ifndef PLANNING_CPP_MOVES_H
#define PLANNING_CPP_MOVES_H


#include "../plan.h"
#include "../vns_interface.h"



/** A simple move that does nothing. Typically used to represent a fix-point in type-safe manner. */
class IdentityMove final : public LocalMove {
public:
    IdentityMove(PPlan p) : LocalMove(p) {}

    /** Cost that would result in applying the move. */
    virtual double utility() override { return base_plan->utility(); };

    /** Total duration that would result in applying the move */
    virtual double duration() override { return base_plan->duration(); };

    virtual bool is_valid() override { return base_plan->is_valid(); }

    /** does nothing */
    void apply_on(PPlan p) override {
    }
};


/** A convenience abstract implementation of LocalMove that will compute cost, duration and additional segments
 * by applying the move an a new plan.
 *
 * Subclassses only need to implement the apply_on() method and invoke the init() method in there constructor. */
struct CloneBasedLocalMove : public LocalMove {
    CloneBasedLocalMove(const PPlan base) : LocalMove(base) {
    }

    /** Cost that would result in applying the move. */
    virtual double utility() override { init(); return _cost; };

    /** Total duration that would result in applying the move */
    virtual double duration() override { init(); return _duration; };

    virtual bool is_valid() override { init(); return _valid; }

private:
    void init() {
        if(!lazily_initialized) {
            PPlan clone = apply_on_new();
            _cost = clone->utility();
            _duration = clone->duration();
            _num_segments = clone->num_segments();
            _valid = clone->is_valid();
            lazily_initialized = true;
        }
    }

    mutable bool lazily_initialized = false;
    mutable double _cost;
    mutable double _duration;
    mutable size_t _num_segments;
    mutable bool _valid;
};

/** Local move that insert a segment at given place in the plan. */
struct Insert final : public CloneBasedLocalMove {
    /** Index of the trajectory in which to perform the insertion. */
    size_t traj_id;

    /** Segment to insert. */
    Segment seg;

    /** Place in the trajectory where this segment should be inserted. */
    size_t insert_loc;

    Insert(PPlan base, size_t traj_id, Segment seg, size_t insert_loc)
            : CloneBasedLocalMove(base),
              traj_id(traj_id),
              seg(seg),
              insert_loc(insert_loc)
    {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(insert_loc <= base->trajectories[traj_id].traj.size());
    }

    void apply_on(PPlan p) override {
        p->insert_segment(traj_id, seg, insert_loc);
    }

    /** Generates an insert move that include the segment at the best place in the given trajectory.
     * Currently, this does not checks the trajectory constraints. */
    static opt<Insert> best_insert(PPlan base, size_t traj_id, Segment seg) {
        ASSERT(traj_id < base->trajectories.size());

        long best_loc = -1;
        double best_dur = 999999;
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
struct Remove final : public CloneBasedLocalMove {
    /** Index of the trajectory in which to perform the removal. */
    size_t traj_id;

    /** Location of the segment to remove within the trajectory */
    size_t rm_id;

    Remove(PPlan base, size_t traj_id, size_t rm_id)
            : CloneBasedLocalMove(base),
              traj_id(traj_id),
              rm_id(rm_id) {
        ASSERT(traj_id < base->trajectories.size());
        ASSERT(rm_id < base->trajectories[traj_id].traj.size());
    }

    void apply_on(PPlan p) override {
        p->erase_segment(traj_id, rm_id);
    }
};

struct SegmentSwap : public CloneBasedLocalMove {
    const size_t traj_id;
    const size_t segment_index;
    const Segment newSegment;

    SegmentSwap(const PPlan &base, size_t traj_id, size_t segment_index, const Segment& replacement)
            : CloneBasedLocalMove(base),
              traj_id(traj_id),
              segment_index(segment_index),
              newSegment(replacement)
    {

        ASSERT(traj_id < base->trajectories.size());
        ASSERT(segment_index < base->trajectories[traj_id].traj.size());
        ASSERT(duration() >= 0)
    }

    void apply_on(PPlan p) override {
        p->replace_segment(traj_id, segment_index, newSegment);
    }
};


struct SegmentRotation final : public CloneBasedLocalMove {
    const size_t traj_id;
    const size_t segment_index;
    const Segment newSegment;
    SegmentRotation(const PPlan &base, size_t traj_id, size_t segment_index, double target_dir)
            : CloneBasedLocalMove(base),
              traj_id(traj_id),
              segment_index(segment_index),
              newSegment(base->uav(traj_id).rotate_on_visibility_center(base->trajectories[traj_id][segment_index],
                                                                        target_dir))
    {
        ASSERT(duration() >= 0)
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
