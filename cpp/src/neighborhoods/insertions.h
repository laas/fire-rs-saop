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
    const double default_segment_length = 50;

    const double default_height = 100;

    const double max_trials;
    explicit OneInsertNbhd(double max_trials = 50) : max_trials(max_trials) {}

    opt<PLocalMove> get_move(PPlan p) override {
        IdentityMove no_move(p);
        opt<PLocalMove> best = {};

        size_t num_tries = 0;
        while(num_tries++ < max_trials) {
            opt<PLocalMove> candidateOpt = get_move_for_random_possible_observation(p);
            if(candidateOpt) {
                // a move was generated
                PLocalMove candidate_move = *candidateOpt;

                if(!candidate_move->is_valid())
                    // move is not valid, discard it
                    continue;

                if(!best && is_better_than(*candidate_move, no_move)) {
                    // no best move, and better than doing nothing
                    best = candidate_move;
                } else if(best && is_better_than(*candidate_move, **best)) {
                    // better than the best move
                    best = candidate_move;
                }
            }
        }
        return best;
    }

private:
    /** this move is better than another if it has a significantly better cost or if it has a similar cost but a strictly better duration */
    bool is_better_than(LocalMove& first, LocalMove& other) {
        return first.utility() < other.utility() -.7
                   || (ALMOST_LESSER_EQUAL(first.utility(), other.utility()) &&  first.duration() < other.duration());
    }

    /** Picks an observation randomly and generates a move that inserts it into the best looking location. */
    opt<PLocalMove> get_move_for_random_possible_observation(PPlan p) {
        ASSERT(!p->trajectories.empty())
        if(p->possible_observations.empty())
            return {};

        /** Select a random point in the pending list */
        const size_t index = rand() % p->possible_observations.size();
        const Point3dTimeWindow pt = p->possible_observations[index];

        /** Pick an angle randomly */
        const double random_angle = drand(0, 2*M_PI);

        /** Waypoint and segment resulting from the random picks */
        const Segment3d random_observation = p->uav(0).observation_segment(
                pt.pt.x, pt.pt.y,
                default_height + (*p->firedata->elevation)(p->firedata->elevation->as_cell(pt.pt)),
                random_angle, default_segment_length);

        opt<Candidate> best;

        /** Try best insert for each subtrajectory in the plan */
        for(size_t i=0; i< p->trajectories.size(); i++) {
            const Trajectory& traj = p->trajectories[i];

            for(size_t insert_loc=traj.first_modifiable(); insert_loc<=traj.last_modifiable()+1; insert_loc++) {

                opt<Segment3d> current_segment = random_observation;
                bool updated = true;
                size_t num_iter = 0;

                // project the segment on firefront until we converge or are enable to project the segment anymore.
                while(current_segment && updated) {
                    // time at which the uav will reach the segment
                    const double time = insert_loc==0 ?
                                            traj.start_time() :
                                            traj.end_time(insert_loc-1) +
                                                        traj.conf.uav.travel_time(traj[insert_loc-1].end, random_observation.start);
                    // back up current segment
                    const Segment3d previous = *current_segment;

                    // project the segment on the firefront.
                    current_segment = p->firedata->project_on_firefront(previous, traj.conf.uav, time);
                    ASSERT(!current_segment || previous.start.dir == current_segment->start.dir) // projection should not change orientation

                    if(current_segment && previous != *current_segment) {
                        // segment has changed due to projection,
                        // set the default orientation to the segment
                        const double direction = default_insertion_angle(traj, insert_loc, *current_segment);
                        current_segment = traj.conf.uav.rotate_on_visibility_center(*current_segment, direction);

                        // start over since the time at which we reach the segment might have changed as well
                        updated = true;
                    } else {
                        updated = false;
                    }
                    ASSERT(num_iter++ <1000); // I assumed that this always converges (very quickly). One can safely put a bound on the number of iterations.
                }
                if(current_segment) {
                    // current segment is valid (i.e. successfully projected on firefront,
                    // create a candidate for it

                    const double additional_flight_time = traj.insertion_duration_cost(insert_loc, *current_segment);

                    // discard candidate that would go over the max flight time.
                    if (traj.duration() + additional_flight_time > traj.conf.max_flight_time)
                        continue;

                    if (!best || additional_flight_time < best->additional_flight_time) {
                        best = Candidate {i, insert_loc, *current_segment, additional_flight_time};
                    }
                }
            }
        }

        /** Return the best, if any */
        if(best)
            return opt<PLocalMove>(make_shared<Insert>(p, best->traj_id, best->segment, best->insert_loc));
        else
            return {};
    }

private:
    struct Candidate {
        size_t traj_id;
        size_t insert_loc;
        Segment3d segment;
        double additional_flight_time;
    };

    double default_insertion_angle(const Trajectory& traj, size_t insertion_loc, const Segment3d& segment) const {
        if(traj.size() == 0) {
            return segment.start.dir;
        } else if(insertion_loc == 0) {
            // align on next segment
            auto dx = traj[insertion_loc].start.x - segment.end.x;
            auto dy = traj[insertion_loc].start.y - segment.end.y;
            return atan2(dy, dx);
        } else if(insertion_loc == traj.size()) {
            // align on previous segment
            auto dx = segment.start.x - traj[insertion_loc-1].end.x;
            auto dy = segment.start.y - traj[insertion_loc-1].end.y;
            return atan2(dy, dx);
        } else {
            // take direction from prev to next segment
            auto dx = traj[insertion_loc].start.x - traj[insertion_loc-1].end.x;
            auto dy = traj[insertion_loc].start.y - traj[insertion_loc-1].end.y;
            return atan2(dy, dx);
        }
    }
};



#endif //PLANNING_CPP_INSERTIONS_H
