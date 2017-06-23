#ifndef PLANNING_CPP_INSERTIONS_H
#define PLANNING_CPP_INSERTIONS_H


#include "../planning.h"

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
        _cost = base->visibility.cost_given_addition(base->trajectories[traj_id].conf->uav, seg);
        _duration = base->duration() + base->trajectories[traj_id].insertion_duration_cost(insert_loc, seg);
    }

    void apply_on(PPlan p) override {
        p->insert_segment(traj_id, seg, insert_loc);
    }

    /** Generates an insert move that include the segment at the best place in the given trajectory.
     * Currently, this does not checks the trajectory constraints. */
    static opt<Insert> best_insert(PPlan base, size_t traj_id, Segment seg, double max_cost=INF) {
        ASSERT(traj_id < base->trajectories.size());

        if(max_cost == INF || max_cost >= base->visibility.cost_given_addition(base->trajectories[traj_id].conf->uav, seg)) {
            long best_loc = -1;
            double best_dur = INF;
            Trajectory& traj = base->trajectories[traj_id];
            for(size_t i=0; i<=traj.traj.size(); i++) {
                const double dur = traj.duration() + traj.insertion_duration_cost(i, seg);
                if(dur <=  traj.conf->max_flight_time) {
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
        } else {
            return {};
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

/** Neighborhood for generating Insert moves.
 *
 * The neighborhood randomly picks a pending point in the visibility structures and returns
 * the best insertion for this point.
 */
struct OneInsertNbhd : public Neighborhood {

    opt<PLocalMove> get_move(PPlan p) override {
        if(p->visibility.interesting_pending.size() == 0) {
            // no candidates left
            return {};
        }
        /** Select a random point in the pending list */
        const size_t index = rand() % p->visibility.interesting_pending.size();
        const Cell pt = p->visibility.interesting_pending[index];

        /** Pick an angle randomly */
        const double angle = ((double) rand() / RAND_MAX) * (M_PI * 2);

        /** Waypoint and segment resulting from the random picks */
        const Waypoint wp = Waypoint(p->visibility.ignitions.x_coords(pt.x), p->visibility.ignitions.y_coords(pt.y), angle);
        const Segment seg = Segment(wp.forward(-30), 30);

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
