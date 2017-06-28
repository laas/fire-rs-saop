#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include "../trajectory.h"
#include "../planning.h"
#include "../vns_interface.h"
#include "moves.h"


/** Interface for objects generating orientation candidates for particular segments in a trajectory.  */
struct OrientationChangeGenerator {

    /** Method that should suggest an orientation for the segment at the given index in the trajectory.
     * If it is not applicable, it should return an empty option. */
    virtual opt<double> get_orientation_change(const Trajectory& traj, size_t seg_id) const = 0;
};

/** Generator the suggests the direction from the predecessor to the successor of a given segment.
 * This is not applicable if the segment is first or last.
 * */
struct MeanOrientationChangeGenerator final : public OrientationChangeGenerator {

    opt<double> get_orientation_change(const Trajectory& traj, size_t seg_id) const override {
        if(seg_id == 0 || seg_id == traj.size()-1)
            return {};

        const double dx = traj[seg_id+1].start.x - traj[seg_id-1].end.x;
        const double dy = traj[seg_id+1].start.y - traj[seg_id-1].end.y;
        const double mean_angle = atan2(dy, dx);
        return mean_angle;
    }
};

/** Suggests a random angle. */
struct RandomOrientationChangeGenerator final : public OrientationChangeGenerator {

    opt<double> get_orientation_change(const Trajectory& traj, size_t seg_id) const override {
        return drand(0, 2*M_PI);
    }
};

/** Combination of all neighborhoods intended to smooth dubins trajectories. */
struct DubinsOptimizationNeighborhood final : public Neighborhood {

    /** Angle generators to be considered. */
    const vector<shared_ptr<OrientationChangeGenerator>> generators;

    /** Maximum number of changes to try before returning. */
    const size_t max_trials;

    DubinsOptimizationNeighborhood(vector<shared_ptr<OrientationChangeGenerator>> generators = default_generators(), size_t max_trials = 100) :
            generators(generators),
            max_trials(max_trials) {
        ASSERT(!generators.empty());
    }

    opt<PLocalMove> get_move(PPlan plan) {
        if(plan->num_segments() == 0)
            return {};

        // Generates local moves until one is improves duration and is valid or the maximum number of trials is reached.
        int num_trials = 0;
        while(num_trials++ < max_trials) {
            // pick a random trajectory in plan.
            const size_t traj_id = rand(0, plan->trajectories.size());
            const Trajectory& traj = plan->trajectories[traj_id];

            if(traj.empty())
                continue; // nothing in this trajectory, go to next trial

            // pick a random segment in the trajectory
            const size_t seg_id = rand(traj.first_modifiable(), traj.last_modifiable()+1);

            // pick an angle generator and generate a candidate angle
            const shared_ptr<OrientationChangeGenerator> generator = generators[rand(0, generators.size())];
            opt<double> optAngle = generator->get_orientation_change(traj, seg_id);

            if(!optAngle)
                continue; // generator not adapted to current segment, go to next trial

            // compute the cost of the change
            const Segment replacement_segment = plan->uav(traj_id).rotate_on_visibility_center(traj[seg_id], *optAngle);
            const double local_duration_cost = traj.replacement_duration_cost(seg_id, replacement_segment);

            // if the orientation change is strictly improving, return it if it is valid
            if(local_duration_cost < -0.0001) {
                PLocalMove move = make_shared<SegmentRotation>(plan, traj_id, seg_id, *optAngle);
                if(move->is_valid())
                    return move;
            }
        }
        // we did not find any duration improving move
        return {};
    }

private:
    static vector<shared_ptr<OrientationChangeGenerator>> default_generators() {
        return vector<shared_ptr<OrientationChangeGenerator>> {
                (shared_ptr<OrientationChangeGenerator>) make_shared<RandomOrientationChangeGenerator>(),
                (shared_ptr<OrientationChangeGenerator>) make_shared<MeanOrientationChangeGenerator>()
        };
    }
};

#endif //PLANNING_CPP_DUBINS_OPTIMIZATION_H
