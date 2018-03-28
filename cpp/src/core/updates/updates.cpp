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

#include "updates.hpp"

namespace SAOP {

    std::string EmptyUpdate::to_string() const {
        return "EmptyUpdate()";
    }

    PReversibleTrajectoriesUpdate SequencedUpdates::apply(Trajectories& p) {
        PReversibleTrajectoriesUpdate first_reverse = first->apply(p);
        PReversibleTrajectoriesUpdate second_reverse = second->apply(p);
        return unique_ptr<SequencedUpdates>(
                new SequencedUpdates(std::move(second_reverse), std::move(first_reverse)));
    }

    std::string SequencedUpdates::to_string() const {
        std::stringstream ss;
        ss << "SequencedUpdates(" << first.get() << ", " << second.get() << ")";
        return ss.str();
    }

    PReversibleTrajectoriesUpdate InsertSegmentUpdate::apply(Trajectories& p) {
        ASSERT(traj_id < p.size());
        ASSERT(at_index <= p[traj_id].size());
        p[traj_id].insert_segment(seg, at_index);
        return unique_ptr<DeleteSegmentUpdate>(new DeleteSegmentUpdate(traj_id, at_index));
    }

    std::string InsertSegmentUpdate::to_string() const {
        std::stringstream ss;
        ss << "InsertSegment(t=" << traj_id << ", s=" << seg << ", i=" << at_index << ")";
        return ss.str();
    }

    PReversibleTrajectoriesUpdate DeleteSegmentUpdate::apply(Trajectories& p) {
        ASSERT(traj_id < p.size());
        ASSERT(at_index < p[traj_id].size());
        Segment3d seg = p[traj_id][at_index].maneuver;
        p[traj_id].erase_segment(at_index);
        return unique_ptr<InsertSegmentUpdate>(new InsertSegmentUpdate(traj_id, seg, at_index));
    }

    std::string DeleteSegmentUpdate::to_string() const {
        std::stringstream ss;
        ss << "DeleteSegment(t=" << traj_id << ", i=" << at_index << ")";
        return ss.str();
    }

    PReversibleTrajectoriesUpdate ReplaceSegmentUpdate::apply(Trajectories& p) {
        ASSERT(traj_id < p.size());
        ASSERT(at_index < p[traj_id].size());
        Segment3d old_seg = p[traj_id][at_index].maneuver;
        p[traj_id].replace_segment(at_index, new_seg);
        return unique_ptr<ReplaceSegmentUpdate>(new ReplaceSegmentUpdate(traj_id, at_index, old_seg));
    }

    std::string ReplaceSegmentUpdate::to_string() const {
        std::stringstream ss;
        ss << "ReplaceSegmentUpdate(t=" << traj_id << ", i=" << at_index << ", s=" << new_seg << ")";
        return ss.str();
    }
}