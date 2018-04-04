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

#ifndef PLANNING_CPP_PLAN_H
#define PLANNING_CPP_PLAN_H

#include <memory>
#include <queue>
#include <stack>

#include "../core/trajectory.hpp"
#include "../core/fire_data.hpp"
#include "../core/raster.hpp"
#include "../core/trajectories.hpp"
#include "../core/updates/updates.hpp"
#include "../ext/json.hpp"

#include "../firemapping/ghostmapper.hpp"

namespace SAOP {

    using json = nlohmann::json;

    struct Plan;
    typedef shared_ptr<Plan> PlanPtr;

    struct Plan {
        TimeWindow time_window; /* Cells outside the range are not considered in possible observations */
        vector<PointTimeWindow> possible_observations;
        vector<PositionTime> observed_previously;

        Plan(vector<TrajectoryConfig> traj_confs, shared_ptr<FireData> fire_data, TimeWindow tw,
             vector<PositionTime> observed_previously = {});

        ~Plan() = default;

        // copy constructor
        Plan(const Plan& plan) = default;

        // move constructor
        Plan(Plan&& plan) = default;

        // copy assignment operator
        Plan& operator=(const Plan& plan) = default;

        // move assignment operator
        Plan& operator=(Plan&& plan) = default;

        json metadata();

        /** A plan is valid iff all trajectories are valid (match their configuration. */
        bool is_valid() const {
            return trajs.is_valid();
        }

        const Trajectories& trajectories() const {
            return trajs;
        }

        const FireData& firedata() const {
            return *fire_data;
        }

        /* Replace plan firedata */
        void firedata(shared_ptr<FireData> fdata) {
            fire_data = fdata;
            utility_recomp_request = true;
        }

        /** Sum of all trajectory durations. */
        double duration() const {
            return trajs.duration();
        }

        /* Utility of the plan */
        double utility() {
            if (utility_recomp_request) {
                GenRaster<double> utility = utility_map();
                auto accumulate_ignoring_nan = [](double a, double b) { return isnan(b) ? a : a + b; };
                utility_cache = std::accumulate(utility.begin(), utility.end(), 0., accumulate_ignoring_nan);
            }
            return utility_cache;
        }

        GenRaster<double> utility_map() const {
            return utility_comp_propagation();
        }

        size_t num_segments() const {
            return trajs.num_segments();
        }

        /** All observations in the plan. Computed by taking the visibility center of all segments.
         * Each observation is tagged with a time, corresponding to the start time of the segment.*/
        vector<PositionTime> observations() const {
            //return observations(time_window);
            return observations_full();
        }

        /** All observations in the plan. Computed assuming we observe at any time, not only when doing a segment*/
        vector<PositionTime> observations_full() const;

        /* Observations done within an arbitrary time window
         */
        vector<PositionTime> observations(const TimeWindow& tw) const;

        /*All the positions observed by the UAV camera*/
        vector<PositionTime> view_trace(const TimeWindow& tw) const;

        /*All the positions observed by the UAV camera*/
        vector<PositionTime> view_trace() const {
            return view_trace(time_window);
        }

        void insert_segment(size_t traj_id, const Segment3d& seg, size_t insert_loc, bool do_post_processing = true);

        void erase_segment(size_t traj_id, size_t at_index, bool do_post_processing = true);

        void replace_segment(size_t traj_id, size_t at_index, const Segment3d& by_segment);

        void
        replace_segment(size_t traj_id, size_t at_index, size_t n_replaced, const std::vector<Segment3d>& segments);

        PReversibleTrajectoriesUpdate update(PReversibleTrajectoriesUpdate u, bool do_post_processing = false);

        void freeze_before(double time);

        void post_process() {
            project_on_fire_front();
            smooth_trajectory();
            utility_recomp_request = true;
        }

        /** Make sure every segment makes an observation, i.e., that the picture will be taken when the fire in traversing the main cell.
         *
         * If this is not the case for a given segment, its is projected on the firefront.
         * */
        void project_on_fire_front();

        /** Goes through all trajectories and erase segments causing very tight loops. */
        void smooth_trajectory();

        /* Get the cells of the Raster
         * */
        template<typename GenRaster>
        static opt<std::vector<Cell>> segment_trace(const Segment3d& segment, double view_width,
                                                    double view_depth, const GenRaster& raster) {
            return SAOP::RasterMapper::segment_trace<GenRaster>(segment, view_width, view_depth, raster);
        }

    private:
        Trajectories trajs;
        shared_ptr<FireData> fire_data;

        /** Constants used for computed the cost associated to a pair of points.
         * The cost is MAX_INDIVIDUAL_COST if the distance between two points
         * is >= MAX_INFORMATIVE_DISTANCE. It is be 0 if the distance is 0 and scales linearly between the two. */
        double MAX_INFORMATIVE_DISTANCE = 500.;

        /** If a point is less than REDUNDANT_OBS_DIST aways from another observation, it useless to observe it.
         * This is defined such that those point are in the visible area when pictured. */
        double REDUNDANT_OBS_DIST = 50.;

        /* Max and min utility values. Visited cells have MIN_UTILITY utility. */
        double MAX_UTILITY = 1.;
        double MIN_UTILITY = 0.;

        bool utility_recomp_request = true;
        double utility_cache = 0.;

        /** Utility map of the plan.
         * The key idea is to sum the distance of all ignited points in the time window to their closest observation.
         **/
        GenRaster<double> utility_comp_radial() const;

        /* Utility map of the plan
         * This algorithm applies regressive utility gains to future ignited cells following the propagation graph.*/
        GenRaster<double> utility_comp_propagation() const;
    };
}

#endif //PLANNING_CPP_PLAN_H
