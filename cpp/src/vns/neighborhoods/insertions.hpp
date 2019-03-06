/* Copyright (c) 2017, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

#ifndef PLANNING_CPP_INSERTIONS_H
#define PLANNING_CPP_INSERTIONS_H

#include "moves.hpp"

namespace SAOP {

    /** Neighborhood for generating Insert moves.
     *
     * The neighborhood randomly picks a pending point in the visibility structures and returns
     * the best insertion for this point.
     */
    struct OneInsertNbhd final : public Neighborhood {

        std::string name() const override {
            return "one-insert";
        }

        const double default_segment_length = 0.;

        const double default_height = 0.;

        const double max_trials;
        const bool select_arbitrary_trajectory;
        const bool select_arbitrary_position;

        explicit OneInsertNbhd(double max_trials,
                               const bool select_arbitrary_trajectory,
                               const bool select_arbitrary_position)
                : max_trials(max_trials),
                  select_arbitrary_trajectory(select_arbitrary_trajectory),
                  select_arbitrary_position(select_arbitrary_position) {
            BOOST_LOG_TRIVIAL(info) << "OneInsertNbhd is inserting waypoints at " << default_height
                                    << " above ground altitude";
        }

        unique_ptr<LocalMove> get_move(PlanPtr p) override {
            UpdateBasedMove no_move(p, unique_ptr<EmptyUpdate>(new EmptyUpdate()));
            unique_ptr<LocalMove> best = {};

            size_t num_tries = 0;
            while (num_tries++ < max_trials) {
                unique_ptr<LocalMove> candidate_move = get_move_for_random_possible_observation(p);
                if (candidate_move) {
                    // a move was generated

                    if (!candidate_move->is_valid())
                        // move is not valid, discard it
                        continue;

                    if (!best && is_better_than(*candidate_move, no_move)) {
                        // no best move, and better than doing nothing
                        best = std::move(candidate_move);
                    } else if (best && is_better_than(*candidate_move, *best)) {
                        // better than the best move
                        best = std::move(candidate_move);
                    }
                }
            }
            return best;
        }

    private:
        /** this move is better than another if it has a significantly better cost or if it has a similar cost but a strictly better duration */
        bool is_better_than(LocalMove& first, LocalMove& other) {
//            return localmove_efficiency(first) > localmove_efficiency(other) && (first.utility() + 1) < other.utility();
            return ((first.utility() + 1) <
                    other.utility());// || (abs(first.utility() - other.utility()) < 1 && (first.duration() < other.duration()));
        }

        double localmove_efficiency(LocalMove& localmove) {
            double delta_duration = localmove.duration() - localmove.base_plan->duration();
            double delta_utility = -(localmove.utility() - localmove.base_plan->utility());

            if (delta_duration > 0 && delta_utility > 0) {
                return delta_utility / delta_duration;
            } else {
                return 0.;
            }
        }

        /** Picks an observation randomly and generates a move that inserts it into the best looking location. */
        unique_ptr<LocalMove> get_move_for_random_possible_observation(PlanPtr p) {
            ASSERT(!p->trajectories().empty());
            if (p->possible_observations.empty())
                return {};

            /** Select a random point in the pending list */
            const size_t index = rand(0, p->possible_observations.size());
            const PointTimeWindow pt = p->possible_observations[index];

            /** Pick an angle randomly */
            const double random_angle = drand(0, 2 * M_PI);

            /** Waypoint and segment resulting from the random picks */
//            const Segment3d random_observation = p->trajectories().uav(0).observation_segment(
//                    pt.pt.x, pt.pt.y,
//                    default_height + (*p->firedata().elevation)(p->firedata().elevation->as_cell(pt.pt)),
//                    random_angle, default_segment_length);
            Segment3d random_observation = p->trajectories().uav(0).observation_segment(
                    pt.pt.x, pt.pt.y,
                    default_height + (*p->firedata().elevation)(p->firedata().elevation->as_cell(pt.pt)),
                    random_angle, default_segment_length);


            opt<Candidate> best;

            // last valid projection made of the random observation.
            // it used to incrementally update the projection to different times,
            // Simple measurements indicate that doing this incrementally reduces
            // the number of steps in "project on_firefront" by a factor ~5
            Segment3d projected_random_observation = random_observation;

            size_t first_traj, last_traj;
            if (select_arbitrary_trajectory) {
                const size_t t = rand(0, p->trajectories().size());
                first_traj = t;
                last_traj = t;
            } else {
                first_traj = 0;
                last_traj = p->trajectories().size() - 1;
            }

            /** Try best insert for each subtrajectory in the plan */
            for (size_t i = first_traj; i <= last_traj; i++) {
                const Trajectory& traj = p->trajectories()[i];

                // FIXME: DO not force a constant altitude
                // This is a workaround for the limitation in z of DubinsWind
                random_observation = Segment3d(
                        projected_random_observation.start.with_z(p->trajectories()[i][0].maneuver.start.z),
                        projected_random_observation.end.with_z(p->trajectories()[i][0].maneuver.start.z));

                // get projected observation closer to the fire front at the beginning of the trajectory
                // this is useful to avoid to projection to follow the same path multiple times to end up on a failure.
                projected_random_observation = p->firedata().project_closest_to_fire_front(random_observation,
                                                                                           traj.conf().uav,
                                                                                           traj.start_time());

                size_t first_insertion_loc, last_insertion_loc;
                if (select_arbitrary_position) {
                    const opt<size_t> loc = traj.random_insertion_id();
                    if (loc) {
                        first_insertion_loc = *loc;
                        last_insertion_loc = *loc;
                    } else {
                        first_insertion_loc = 1;
                        last_insertion_loc = 0;
                    }
                } else {
                    first_insertion_loc = traj.insertion_range_start();
                    last_insertion_loc = traj.insertion_range_end();
                }

                for (size_t insert_loc = first_insertion_loc; insert_loc <= last_insertion_loc; insert_loc++) {

                    opt<Segment3d> current_segment = get_projection(p, projected_random_observation, i, insert_loc);

                    if (current_segment) {
                        // save to be able to use it as a base for the following projections
                        projected_random_observation = *current_segment;
                        // current segment is valid (i.e. successfully projected on fire front,
                        // create a candidate for it

                        const double additional_flight_time = traj.insertion_duration_cost(insert_loc,
                                                                                           *current_segment);

                        // discard candidate that would go over the max flight time.
                        if (traj.duration() + additional_flight_time > traj.conf().max_flight_time)
                            continue;

                        if (!best || additional_flight_time < best->additional_flight_time) {
                            best = Candidate{i, insert_loc, *current_segment, additional_flight_time};
                        }
                    }
                }
            }

            /** Return the best, if any */
            if (best) {
//                return unique_ptr<UpdateBasedMove>(new UpdateBasedMove(p, unique_ptr<InsertSegmentUpdate>(
//                        new InsertSegmentUpdate(best->traj_id, best->segment, best->insert_loc))));
                return unique_ptr<Insert>(new Insert(p, best->traj_id, best->segment, best->insert_loc));
            } else {
                return {};
            };
        }

    private:
        struct Candidate {
            size_t traj_id;
            size_t insert_loc;
            Segment3d segment;
            double additional_flight_time;
        };

        double
        default_insertion_angle(const Trajectory& traj, size_t insertion_loc, const Segment3d& segment) const {
            if (traj.size() == 0) {
                return segment.start.dir;
            } else if (insertion_loc == 0) {
                // align on next segment
                auto dx = traj[insertion_loc].maneuver.start.x - segment.end.x;
                auto dy = traj[insertion_loc].maneuver.start.y - segment.end.y;
                return atan2(dy, dx);
            } else if (insertion_loc == traj.size()) {
                // align on previous segment
                auto dx = segment.start.x - traj[insertion_loc - 1].maneuver.end.x;
                auto dy = segment.start.y - traj[insertion_loc - 1].maneuver.end.y;
                return atan2(dy, dx);
            } else {
                // take direction from prev to next segment
                auto dx = traj[insertion_loc].maneuver.start.x - traj[insertion_loc - 1].maneuver.end.x;
                auto dy = traj[insertion_loc].maneuver.start.y - traj[insertion_loc - 1].maneuver.end.y;
                return atan2(dy, dx);
            }
        }

        /** Produces a new segment for observation. The segment is such that:
         * if inserted after the insert_loc^th observation of the traj_id^th trajectory, the UAV will reach it when
         * the underneath cell is on fire.
         * If there is no such cell, an empty option is returned. */
        opt<Segment3d>
        get_projection(const PlanPtr& p, const Segment3d to_project, size_t traj_id, size_t insert_loc) {
            const Trajectory& traj = p->trajectories()[traj_id];
            // start iteration from last valid projection made.
            opt<Segment3d> current_segment = to_project;
            bool updated = true;
            size_t num_iter = 0;

            // project the segment on firefront until we converge or are unable to project the segment anymore.
            while (current_segment && updated) {
                // time at which the uav will reach the segment
                const double time = insert_loc == 0 ?
                                    traj.start_time() :
                                    traj.end_time(insert_loc - 1) +
                                    traj.conf().uav.travel_time(traj[insert_loc - 1].maneuver.end,
                                                                to_project.start);
                // back up current segment
                const Segment3d previous = *current_segment;

                // project the segment on the firefront.
                current_segment = p->firedata().project_on_firefront(previous, traj.conf().uav, time);
                ASSERT(!current_segment ||
                       previous.start.dir ==
                       current_segment->start.dir); // projection should not change orientation

                if (current_segment && previous != *current_segment) {
                    // segment has changed due to projection,
                    // set the default orientation to the segment
                    const double direction = default_insertion_angle(traj, insert_loc, *current_segment);
                    current_segment = traj.conf().uav.rotate_on_visibility_center(*current_segment, direction);

                    // start over since the time at which we reach the segment might have changed as well
                    updated = true;
                } else {
                    updated = false;
                }
                ASSERT(num_iter++ <
                       1000); // This should always converges (very quickly). One can safely put a bound on the number of iterations.
            }
            return current_segment;
        }
    };
}

#endif //PLANNING_CPP_INSERTIONS_H
