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
    const double default_segment_length = 30;

    opt<PLocalMove> get_move(PPlan p) override {
        ASSERT(!p->trajectories.empty())
        if(p->possible_observations.empty())
            return {};

        /** Select a random point in the pending list */
        const size_t index = rand() % p->possible_observations.size();
        const PointTimeWindow pt = p->possible_observations[index];

        /** Pick an angle randomly */
        const double angle = drand(0, 2*M_PI);

        /** Waypoint and segment resulting from the random picks */
        const Segment seg = p->uav(0).observation_segment(pt.pt.x, pt.pt.y, angle, default_segment_length);

        opt<Candidate> best;

        /** Try best insert for each subtrajectory in the plan */
        for(size_t i=0; i< p->trajectories.size(); i++) {
            const Trajectory& traj = p->trajectories[i];

            for(size_t insert_loc=traj.first_modifiable(); insert_loc<=traj.last_modifiable()+1; insert_loc++) {
                // estimated time at which the move can be inserted
                const double time = insert_loc == 0 ? traj.start_time() : traj.end_time(insert_loc-1);

                // project the random segment on the firefront for this time
                auto segmentOpt = p->fire->project_on_firefront(seg, traj.conf.uav, time);
                if(!segmentOpt)
                    continue; // no projection, go to next candidate
                auto segment = *segmentOpt;

                // Set a default orientation of the segment base preceding/following segment.
                // TODO: provide better default for those orientations
                if(insert_loc >= 1) {
                    // align on previous segment
                    auto dx = segment.start.x - traj[insert_loc-1].end.x;
                    auto dy = segment.start.y - traj[insert_loc-1].end.y;
                    auto angle = atan2(dy, dx);
                    segment = traj.conf.uav.rotate_on_visibility_center(segment, angle);
                } else if(traj.size() > 0){
                    // align on next segment
                    auto dx = traj[insert_loc].start.x - segment.end.x;
                    auto dy = traj[insert_loc].start.y - segment.end.y;
                    auto angle = atan2(dy, dx);
                    segment = traj.conf.uav.rotate_on_visibility_center(segment, angle);
                }
                double additional_flight_time = traj.insertion_duration_cost(insert_loc, segment);

                // discard candidate that would go over the max flight time.
                if(traj.duration() + additional_flight_time > traj.conf.max_flight_time)
                    continue;

                if(!best || additional_flight_time < best->additional_flight_time) {
                    best = Candidate { i, insert_loc, segment, additional_flight_time };
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
        Segment segment;
        double additional_flight_time;
    };
};



#endif //PLANNING_CPP_INSERTIONS_H
