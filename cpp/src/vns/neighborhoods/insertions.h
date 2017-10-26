#ifndef PLANNING_CPP_INSERTIONS_H
#define PLANNING_CPP_INSERTIONS_H

#include "moves.h"

/** Neighborhood for generating Insert moves.
 *
 * The neighborhood randomly picks a pending point in the visibility structures and returns
 * the best insertion for this point.
 */
struct OneInsertNbhd final : public Neighborhood {

    std::string name() const override {
        return "one-insert";
    }

    const double default_segment_length = 50;

    const double default_height = 100;

    const double max_trials;
    const bool select_arbitrary_trajectory;
    const bool select_arbitrary_position;
    explicit OneInsertNbhd(double max_trials,
                           const bool select_arbitrary_trajectory,
                           const bool select_arbitrary_position)
            : max_trials(max_trials),
              select_arbitrary_trajectory(select_arbitrary_trajectory),
              select_arbitrary_position(select_arbitrary_position) {}

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
        ASSERT(!p->core.empty())
        if(p->possible_observations.empty())
            return {};

        /** Select a random point in the pending list */
        const size_t index = rand(0, p->possible_observations.size());
        const Point3dTimeWindow pt = p->possible_observations[index];

        /** Pick an angle randomly */
        const double random_angle = drand(0, 2*M_PI);

        /** Waypoint and segment resulting from the random picks */
        const Segment3d random_observation = p->core.uav(0).observation_segment(
                pt.pt.x, pt.pt.y,
                default_height + (*p->firedata->elevation)(p->firedata->elevation->as_cell(pt.pt)),
                random_angle, default_segment_length);

        opt<Candidate> best;

        // last valid projection made of the random observation.
        // it used to incrementally update the projection to different times,
        // Simple measurements indicate that doing this incrementally reduces
        // the number of steps in "project on_firefront" by a factor ~5
        Segment3d projected_random_observation = random_observation;

        size_t first_traj, last_traj;
        if(select_arbitrary_trajectory) {
            const size_t t = rand(0, p->core.size());
            first_traj = t;
            last_traj = t;
        } else {
            first_traj = 0;
            last_traj = p->core.size()-1;
        }

        /** Try best insert for each subtrajectory in the plan */
        for(size_t i=first_traj; i <= last_traj; i++) {
            const Trajectory& traj = p->core[i];

            // get projected observation closer to the fire front at the beginning of the trajectory
            // this is useful to avoid to projection to follow the same path multiple times to end up on a failure.
            projected_random_observation = p->firedata->project_closest_to_fire_front(random_observation, traj.conf.uav, traj.start_time());

            size_t first_insertion_loc, last_insertion_loc;
            if(select_arbitrary_position) {
                const size_t loc = rand(traj.first_modifiable(), traj.last_modifiable()+2);
                first_insertion_loc = loc;
                last_insertion_loc = loc;
            } else {
                first_insertion_loc = traj.first_modifiable();
                last_insertion_loc = traj.last_modifiable() +1;
            }

            for(size_t insert_loc=first_insertion_loc; insert_loc<=last_insertion_loc; insert_loc++) {

                opt<Segment3d> current_segment = get_projection(p, projected_random_observation, i, insert_loc);

                if(current_segment) {
                    // save to be able to use it as a base for the following projections
                    projected_random_observation = *current_segment;
                    // current segment is valid (i.e. successfully projected on fire front,
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

    /** Produces a new segment for observation. The segment is such that:
     * if inserted after the insert_loc^th observation of the traj_id^th trajectory, the UAV will reach it when
     * the underneath cell is on fire.
     * If there is no such cell, an empty option is returned. */
    opt<Segment3d> get_projection(const PPlan p, const Segment3d to_project, size_t traj_id, size_t insert_loc) {
        const Trajectory& traj = p->core[traj_id];
        // start iteration from last valid projection made.
        opt<Segment3d> current_segment = to_project;
        bool updated = true;
        size_t num_iter = 0;

        // project the segment on firefront until we converge or are unable to project the segment anymore.
        while(current_segment && updated) {
            // time at which the uav will reach the segment
            const double time = insert_loc==0 ?
                                traj.start_time() :
                                traj.end_time(insert_loc-1) +
                                traj.conf.uav.travel_time(traj[insert_loc-1].end, to_project.start);
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
            ASSERT(num_iter++ <1000); // This should always converges (very quickly). One can safely put a bound on the number of iterations.
        }
        return current_segment;
    }
};



#endif //PLANNING_CPP_INSERTIONS_H
