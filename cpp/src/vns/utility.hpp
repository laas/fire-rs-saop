/* Copyright (c) 2017-2019, CNRS-LAAS
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
#ifndef PLANNING_CPP_UTILITY_HPP
#define PLANNING_CPP_UTILITY_HPP

#include <utility>
#include <vector>

#include "../core/raster.hpp"
#include "../core/trajectories.hpp"
#include "../core/waypoint.hpp"
#include "../core/fire_data.hpp"

namespace SAOP {
    class Utility {

    public:
        Utility(GenRaster<double> initial_utility, std::shared_ptr<FireData> firedata)
                : base_utility(std::move(initial_utility)), fire_data(std::move(firedata)) {}

        double utility() const;

        GenRaster<double> utility_map() const;

        GenRaster<double> initial_utility() const;

        void reset(Trajectories trajs);

        void reset(std::shared_ptr<FireData> firedata);



    private:
        /* Initial utility map from which observations extract utility */
        GenRaster<double> base_utility;

        /* Firedata used to rectrict observation scope */
        std::shared_ptr<FireData> fire_data;

        /* Max and min utility values. Visited cells have MIN_UTILITY utility. */
        static constexpr double MAX_UTILITY = 1.;
        static constexpr double MIN_UTILITY = 0.;

        /* As utility computation is lazy, mutable cache variables are needed because of const members*/
        mutable opt<double> utility_cache = {};
        mutable opt<GenRaster<double>> utility_map_cache = {};

        opt<Trajectories> trajectories = {};

        static std::vector<std::pair<Position3dTime, Position3dTime>> straight_segments_of(const Trajectory& traj);
        /** Utility map of the plan.
         * Extract the utility from observation trace rectangles.
         **/
        GenRaster<double> utility_impl_trace() const;

    };
}

#endif //PLANNING_CPP_UTILITY_HPP
