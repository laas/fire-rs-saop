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

#ifndef CORE_STRUCTURES_UPDATES_H
#define CORE_STRUCTURES_UPDATES_H

#include "../trajectories.hpp"

using namespace std;

namespace SAOP {

    struct ReversibleTrajectoriesUpdate;
    typedef unique_ptr<ReversibleTrajectoriesUpdate> PReversibleTrajectoriesUpdate;

    struct DeleteSegmentUpdate;

    struct ReversibleTrajectoriesUpdate {

        virtual std::string to_string() const = 0;

        friend std::ostream& operator<<(std::ostream& stream, const ReversibleTrajectoriesUpdate& u) {
            return stream << u.to_string();
        }

        virtual PReversibleTrajectoriesUpdate apply(Trajectories& p) = 0;
    };

    struct EmptyUpdate final : public ReversibleTrajectoriesUpdate {

        std::string to_string() const override;

        PReversibleTrajectoriesUpdate apply(Trajectories& p) override {
            return unique_ptr<EmptyUpdate>(new EmptyUpdate());
        }

    };

    struct SequencedUpdates final : public ReversibleTrajectoriesUpdate {
        PReversibleTrajectoriesUpdate first;
        PReversibleTrajectoriesUpdate second;

        SequencedUpdates(PReversibleTrajectoriesUpdate first, PReversibleTrajectoriesUpdate second)
                : first(std::move(first)),
                  second(std::move(second)) {}

        std::string to_string() const override;

        PReversibleTrajectoriesUpdate apply(Trajectories& p) override;
    };

    struct ReplaceSegmentUpdate : public ReversibleTrajectoriesUpdate {
        /** Index of the trajectory in which to perform the insertion. */
        const size_t traj_id;

        /** Index of the replaced segment. */
        const size_t at_index;

        /** Replacement segment */
        const Segment3d new_seg;

        ReplaceSegmentUpdate(size_t traj_id, size_t at_index, const Segment3d& new_seg)
                : traj_id(traj_id), at_index(at_index), new_seg(new_seg) {};

        std::string to_string() const override;

        PReversibleTrajectoriesUpdate apply(Trajectories& p) override;
    };

    struct InsertSegmentUpdate final : public ReversibleTrajectoriesUpdate {
        /** Index of the trajectory in which to perform the insertion. */
        const size_t traj_id;

        /** Segment to insert. */
        const Segment3d seg;

        /** Place in the trajectory where this segment should be inserted. */
        const size_t at_index;

        InsertSegmentUpdate(size_t traj_id, const Segment3d& seg, size_t at_index) : traj_id(traj_id), seg(seg),
                                                                                     at_index(at_index) {}

        std::string to_string() const override;

        PReversibleTrajectoriesUpdate apply(Trajectories& p) override;
    };

    struct DeleteSegmentUpdate final : public ReversibleTrajectoriesUpdate {

        /** Index of the trajectory in which to perform the insertion. */
        const size_t traj_id;

        /** Place in the trajectory where this segment should be inserted. */
        const size_t at_index;

        DeleteSegmentUpdate(const size_t traj_id, const size_t at_index) : traj_id(traj_id), at_index(at_index) {}

        std::string to_string() const override;

        PReversibleTrajectoriesUpdate apply(Trajectories& p) override;
    };
}

#endif //PROJECT_TRAJECTORIES_H
