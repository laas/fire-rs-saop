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

#ifndef PLANNING_CPP_MOVES_H
#define PLANNING_CPP_MOVES_H


#include "../plan.hpp"
#include "../vns_interface.hpp"
#include "../../core/updates/updates.hpp"

namespace SAOP {

/** A simple move that does nothing. Typically used to represent a fix-point in type-safe manner. */
    class IdentityMove final : public LocalMove {
    public:
        explicit IdentityMove(PlanPtr p) : LocalMove(p) {}

        /** Cost that would result in applying the move. */
        double utility() override { return base_plan->utility(); };

        /** Total duration that would result in applying the move */
        double duration() override { return base_plan->duration(); };

        bool is_valid() override { return base_plan->is_valid(); }

        /** does nothing */
        void apply_on(PlanPtr p) override {
        }
    };


/** A convenience abstract implementation of LocalMove that will compute cost, duration and additional segments
 * by applying the move an a new plan.
 *
 * Subclassses only need to implement the apply_on() method and invoke the init() method in there constructor. */
    struct CloneBasedLocalMove : public LocalMove {
        explicit CloneBasedLocalMove(PlanPtr base) : LocalMove(base) {}

        /** Cost that would result in applying the move. */
        double utility() override {
            init();
            return _cost;
        };

        /** Total duration that would result in applying the move */
        double duration() override {
            init();
            return _duration;
        };

        bool is_valid() override {
            init();
            return _valid;
        }

    private:
        void init() {
            if (!lazily_initialized) {
                PlanPtr clone = apply_on_new();
                _cost = clone->utility();
                _duration = clone->duration();
                _num_segments = clone->num_segments();
                _valid = clone->is_valid();
                lazily_initialized = true;
            }
        }

        mutable bool lazily_initialized = false;
        mutable double _cost = 0;
        mutable double _duration = 0;
        mutable size_t _num_segments = 0;
        mutable bool _valid = false;
    };

    struct UpdateBasedMove : public LocalMove {
        UpdateBasedMove(PlanPtr base, PReversibleTrajectoriesUpdate update)
                : LocalMove(std::move(base)), update(std::move(update)) {}

        /** Cost that would result in applying the move. */
        double utility() override {
            init();
            return _cost;
        };

        /** Total duration that would result in applying the move */
        double duration() override {
            init();
            return _duration;
        };

        bool is_valid() override {
            init();
            return _valid;
        }

    protected:
        void apply_on(PlanPtr target) override {
            if (update) {
                base_plan->update(std::move(update));
                update.reset();
            } else {
                std::cerr << "An UpdateBasedMove can be applied only once." << endl;
            }
        }

    private:
        PReversibleTrajectoriesUpdate update;

        mutable bool lazily_initialized = false;
        mutable double _cost = 0;
        mutable double _duration = 0;
        mutable size_t _num_segments = 0;
        mutable bool _valid = false;

        void init() {
            if (!lazily_initialized || update != nullptr) {
                double duration = base_plan->duration();
                // Do update
                update = std::move(base_plan->update(std::move(update)));
                _cost = base_plan->utility();
                _duration = base_plan->duration();
                _num_segments = base_plan->num_segments();
                _valid = base_plan->is_valid();
                lazily_initialized = true;
                // Revert update
                update = std::move(base_plan->update(std::move(update)));
                ASSERT(ALMOST_EQUAL(duration, base_plan->duration()));
            }
        }
    };

/** Local move that insert a segment at given place in the plan. */
    struct Insert final : public CloneBasedLocalMove {
        /** Index of the trajectory in which to perform the insertion. */
        size_t traj_id;

        /** Segment to insert. */
        Segment3d seg;

        /** Place in the trajectory where this segment should be inserted. */
        size_t insert_loc;

        Insert(PlanPtr base, size_t traj_id, Segment3d& seg, size_t insert_loc)
                : CloneBasedLocalMove(base),
                  traj_id(traj_id), seg(seg), insert_loc(insert_loc) {
            ASSERT(traj_id < base->trajectories.size());
            ASSERT(insert_loc <= base->trajectories[traj_id].size());
        }

        void apply_on(PlanPtr p) override {
            p->insert_segment(traj_id, seg, insert_loc);
        }

        /** Generates an insert move that include the segment at the best place in the given trajectory.
         * Currently, this does not checks the trajectory constraints. */
        static opt<Insert> best_insert(PlanPtr base, size_t traj_id, Segment3d& seg) {
            ASSERT(traj_id < base->trajectories.size());

            long best_loc = -1;
            double best_dur = 999999;
            Trajectory& traj = base->trajectories[traj_id];
            for (size_t i = 0; i <= traj.size(); i++) {
                const double dur = traj.duration() + traj.insertion_duration_cost(i, seg);
                if (dur <= traj.conf().max_flight_time) {
                    if (best_dur > dur) {
                        best_dur = dur;
                        best_loc = i;
                    }
                }
            }
            if (best_loc < 0) {
                return {};
            } else {
                return Insert(base, traj_id, seg, (size_t) best_loc);
            }
        }

        /** Tries to insert each of the segments at the best place in the given trajectory of the given plan */
        static PlanPtr smart_insert(PlanPtr& base, size_t traj_id, vector<Segment3d> segments) {
            opt<PlanPtr> current = base;
            for (auto it = segments.begin(); it != segments.end() && current; it++) {
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

        Remove(PlanPtr base, size_t traj_id, size_t rm_id)
                : CloneBasedLocalMove(base),
                  traj_id(traj_id), rm_id(rm_id) {
            ASSERT(traj_id < base->trajectories.size());
            ASSERT(rm_id < base->trajectories[traj_id].size());
        }

        void apply_on(PlanPtr p) override {
            p->erase_segment(traj_id, rm_id);
        }
    };

    struct SegmentReplacement : public CloneBasedLocalMove {
        const size_t traj_id;
        const size_t segment_index;
        const size_t n_replaced;
        const std::vector<Segment3d> replacements;

        SegmentReplacement(PlanPtr base, size_t traj_id, size_t segment_index, size_t n_replaced,
                           const std::vector<Segment3d>& replacements)
                : CloneBasedLocalMove(base),
                  traj_id(traj_id), segment_index(segment_index), n_replaced(n_replaced), replacements(replacements) {
            ASSERT(n_replaced > 0);
            ASSERT(traj_id < base->trajectories.size());
            ASSERT(segment_index + n_replaced - 1 < base->trajectories[traj_id].size());
            ASSERT(replacements.size() > 0);
        }

        void apply_on(PlanPtr p) override {
            p->replace_segment(traj_id, segment_index, n_replaced, replacements);
        }
    };


    struct SegmentRotation final : public CloneBasedLocalMove {
        const size_t traj_id;
        const size_t segment_index;
        const Segment3d newSegment;

        SegmentRotation(PlanPtr base, size_t traj_id, size_t segment_index, double target_dir)
                : CloneBasedLocalMove(base),
                  traj_id(traj_id), segment_index(segment_index),
                  newSegment(base->trajectories.uav(traj_id).rotate_on_visibility_center(base->trajectories[traj_id][segment_index].maneuver,
                                                                                 target_dir)) {
            ASSERT(duration() >= 0);
        }

        void apply_on(PlanPtr p) override {
            p->replace_segment(traj_id, segment_index, newSegment);
        }

        bool applicable() const {
            Trajectory& t = base_plan->trajectories[traj_id];
            return t.duration() + t.replacement_duration_cost(segment_index, newSegment) <= t.conf().max_flight_time;
        }
    };
}
#endif //PLANNING_CPP_MOVES_H
