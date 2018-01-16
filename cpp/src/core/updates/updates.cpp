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

    PReversibleTrajectoriesUpdate SequencedUpdates::apply(Trajectories& p) {
        PReversibleTrajectoriesUpdate first_reverse = first->apply(p);
        PReversibleTrajectoriesUpdate second_reverse = second->apply(p);
        return make_shared<SequencedUpdates>(second_reverse, first_reverse);
    }

    PReversibleTrajectoriesUpdate InsertSegment::apply(Trajectories& p) {
        ASSERT(traj_id < p.trajectories.size());
        ASSERT(insert_loc <= p.trajectories[traj_id].size());
        p.trajectories[traj_id].insert_segment(seg, insert_loc);
        return make_shared<DeleteSegment>(traj_id, insert_loc);
    }

    PReversibleTrajectoriesUpdate DeleteSegment::apply(Trajectories& p) {
        ASSERT(traj_id < p.trajectories.size());
        ASSERT(at_index < p.trajectories[traj_id].size());
        Segment3d seg = p.trajectories[traj_id][at_index].maneuver;
        p.trajectories[traj_id].erase_segment(at_index);
        return make_shared<InsertSegment>(traj_id, seg, at_index);
    }
}