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

#ifndef PLANNING_CPP_DUBINS_OPTIMIZATION_H
#define PLANNING_CPP_DUBINS_OPTIMIZATION_H

#include <cmath>
#include "../../core/trajectory.hpp"
#include "../vns_interface.hpp"
#include "moves.hpp"

namespace SAOP {

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
            if (seg_id == 0 || seg_id == traj.size() - 1)
                return {};

            const double dx = traj[seg_id + 1].start.x - traj[seg_id - 1].end.x;
            const double dy = traj[seg_id + 1].start.y - traj[seg_id - 1].end.y;
            const double mean_angle = atan2(dy, dx);
            return mean_angle;
        }
    };

/** Suggests a random angle. */
    struct RandomOrientationChangeGenerator final : public OrientationChangeGenerator {

        opt<double> get_orientation_change(const Trajectory& traj, size_t seg_id) const override {
            return remainder(traj[seg_id].start.dir + drand(-M_PI_4, M_PI_4), 2 * M_PI);
        }
    };

/* Suggest flipping the segment*/
    struct FlipOrientationChangeGenerator final : public OrientationChangeGenerator {

        opt<double> get_orientation_change(const Trajectory& traj, size_t seg_id) const override {
            return remainder(traj[seg_id].start.dir + M_PI, 2 * M_PI);
        }
    };

/** Combination of all neighborhoods intended to smooth dubins trajectories. */
    struct DubinsOptimizationNeighborhood final : public Neighborhood {

        std::string name() const override {
            return "dubins-opt";
        }

        /** Angle generators to be considered. */
        const vector<shared_ptr<OrientationChangeGenerator>> generators;

        /** Maximum number of changes to try before returning. */
        const size_t max_trials;

        explicit DubinsOptimizationNeighborhood(
                vector<shared_ptr<OrientationChangeGenerator>> generators = default_generators(),
                size_t max_trials = 10) :
                generators(generators),
                max_trials(max_trials) {
            ASSERT(!generators.empty());
        }

        opt<PLocalMove> get_move(PlanPtr plan) {
            if (plan->num_segments() == 0)
                return {};

            // Generates local moves until one is improves duration and is valid or the maximum number of trials is reached.
            size_t num_trials = 0;
            while (num_trials++ < max_trials) {
                // pick a random trajectory in plan.
                const size_t traj_id = rand(0, plan->core.size());
                const Trajectory& traj = plan->core[traj_id];

                // pick a random segment in the trajectory
                const opt<size_t> opt_seg_id = traj.get_random_modifiable_id();

                if (!opt_seg_id) {
                    continue;
                }
                const size_t seg_id = *opt_seg_id;

                // pick an angle generator and generate a candidate angle
                const shared_ptr<OrientationChangeGenerator> generator = generators[rand(0, generators.size())];
                opt<double> optAngle = generator->get_orientation_change(traj, seg_id);

                if (!optAngle)
                    continue; // generator not adapted to current segment, go to next trial

                // compute the utility of the change
                const Segment3d replacement_segment = plan->core.uav(traj_id).rotate_on_visibility_center(traj[seg_id],
                                                                                                          *optAngle);
                const double local_duration_cost = traj.replacement_duration_cost(seg_id, replacement_segment);


                // if the duration is improving and the utility doesn't get worse then return the move
                if (local_duration_cost < -1) {
                    PLocalMove move = make_shared<SegmentRotation>(plan, traj_id, seg_id, *optAngle);
                    if (move->is_valid())
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
                    //(shared_ptr<OrientationChangeGenerator>) make_shared<MeanOrientationChangeGenerator>(),
                    (shared_ptr<OrientationChangeGenerator>) make_shared<FlipOrientationChangeGenerator>()
            };
        }
    };
}
#endif //PLANNING_CPP_DUBINS_OPTIMIZATION_H
