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


#include "../core/trajectory.hpp"
#include "../core/fire_data.hpp"
#include "../core/raster.hpp"
#include "../ext/json.hpp"
#include "../core/trajectories.hpp"

#include "../firemapping/ghostmapper.hpp"

using json = nlohmann::json;

using namespace std;

namespace SAOP {

    struct Plan;
    typedef shared_ptr<Plan> PlanPtr;

    struct Plan {
        TimeWindow time_window; /* Cells outside the range are not considered in possible observations */
        Trajectories trajectories;
        shared_ptr<FireData> firedata;
        vector<PointTimeWindow> possible_observations;
        vector<PositionTime> observed_previously;

        Plan(const Plan& plan) = default;

        Plan(vector<TrajectoryConfig> traj_confs, shared_ptr<FireData> fire_data, TimeWindow tw,
             vector<PositionTime> observed_previously = {})
                : time_window(tw), trajectories(traj_confs), firedata(std::move(fire_data)),
                  observed_previously(observed_previously) {
            for (auto conf : traj_confs) {
                ASSERT(conf.start_time >= time_window.start && conf.start_time <= time_window.end);
            }

            std::vector<Cell> obs_prev_cells;
            obs_prev_cells.reserve(observed_previously.size());
            std::transform(observed_previously.begin(), observed_previously.end(),
                           std::back_inserter(obs_prev_cells),
                           [this](const PositionTime& pt) -> Cell { return firedata->ignitions.as_cell(pt.pt); });

            for (size_t x = 0; x < firedata->ignitions.x_width; x++) {
                for (size_t y = 0; y < firedata->ignitions.y_height; y++) {
                    const double t = firedata->ignitions(x, y);
                    if (time_window.start <= t && t <= time_window.end) {
                        Cell c{x, y};

                        // If the cell is in the observed_previously list, do not add it to possible_observations
                        if (std::find(obs_prev_cells.begin(), obs_prev_cells.end(), c)
                            == obs_prev_cells.end()) {
                            possible_observations.push_back(
                                    PointTimeWindow{firedata->ignitions.as_position(c),
                                                    {firedata->ignitions(c), firedata->traversal_end(c)}});
                        }
                    }
                }
            }
        }

        json metadata() const {
            json j;
            j["duration"] = duration();
            j["utility"] = utility();
            j["num_segments"] = num_segments();
            j["trajectories"] = json::array();
            for (auto& t : trajectories.trajectories) {
                json jt;
                jt["duration"] = t.duration();
                jt["max_duration"] = t.conf().max_flight_time;
                jt["num_segments"] = t.size();
                jt["start_time"] = t.start_time();
                jt["end_time"] = t.end_time();
                j["trajectories"].push_back(jt);
            }
            return j;
        }

        /** A plan is valid iff all trajectories are valid (match their configuration. */
        bool is_valid() const {
            return trajectories.is_valid();
        }

        /** Sum of all trajectory durations. */
        double duration() const {
            return trajectories.duration();
        }

        /** Cost of the plan.
         * The key idea is to sum the distance of all ignited points in the time window to their closest observation.
         **/
        double utility() const {
            vector<PositionTime> done_obs = observations();
            double global_cost = 0;
            for (const PointTimeWindow& possible_obs : possible_observations) {
                double min_dist = pow(MAX_INFORMATIVE_DISTANCE, 2);
                // find the closest observation.
                for (const PositionTime& obs : done_obs) {
                    min_dist = min(min_dist, possible_obs.pt.dist_squared(obs.pt));
                }
                // utility is based on the minimal distance to the observation and normalized such that
                // utility = 0 if min_dist <= REDUNDANT_OBS_DIST
                // utility = 1 if min_dist = (MAX_INFORMATIVE_DISTANCE - REDUNDANT_OBS_DIST
                // evolves linearly in between.
                const double self_cost = (max(sqrt(min_dist), REDUNDANT_OBS_DIST) - REDUNDANT_OBS_DIST) /
                                         (MAX_INFORMATIVE_DISTANCE - REDUNDANT_OBS_DIST);
                global_cost += self_cost;
            }
            return global_cost;
        }

        size_t num_segments() const {
            return trajectories.num_segments();
        }

        /** All observations in the plan. Computed by taking the visibility center of all segments.
         * Each observation is tagged with a time, corresponding to the start time of the segment.*/
        vector<PositionTime> observations() const {
            return observations(time_window);
        }

        /** All observations in the plan. Computed assuming we observe at any time, not only when doing a segment*/
        vector<PositionTime> observations_full() const {
            std::vector<PositionTime> result = {};
            for (auto& tr: trajectories.trajectories) {
                GhostFireMapper<double> gfm = GhostFireMapper<double>(firedata);
                auto wp_and_t = tr.sampled_with_time(50);
                auto obs = gfm.observed_fire_locations(std::get<0>(wp_and_t), std::get<1>(wp_and_t), tr.conf().uav);
                result.insert(result.end(), obs.begin(), obs.end());
            }
            return result;
        }

        /* Observations done within an arbitrary time window
         */
        vector<PositionTime> observations(const TimeWindow& tw) const {
            vector<PositionTime> obs = std::vector<PositionTime>(observed_previously);
            for (auto& traj : trajectories.trajectories) {
                UAV drone = traj.conf().uav;
                for (size_t seg_id = 0; seg_id < traj.size(); seg_id++) {
                    const Segment3d& seg = traj[seg_id].maneuver;

                    double obs_time = traj.start_time(seg_id);
                    double obs_end_time = traj.end_time(seg_id);
                    TimeWindow seg_tw = TimeWindow{obs_time, obs_end_time};
                    if (tw.contains(seg_tw)) {
                        opt<std::vector<Cell>> opt_cells = segment_trace(seg, drone.view_depth, drone.view_width,
                                                                         firedata->ignitions);
                        if (opt_cells) {
                            for (const auto& c : *opt_cells) {
                                if (firedata->ignitions(c) <= obs_time && obs_time <= firedata->traversal_end(c)) {
                                    // If the cell is observable, add it to the observations list
                                    obs.push_back(
                                            PositionTime{firedata->ignitions.as_position(c), traj.start_time(seg_id)});
                                }
                            }
                        }
                    }
                }
            }
            return obs;
        }

        /*All the positions observed by the UAV camera*/
        vector<PositionTime> view_trace(const TimeWindow& tw) const {
            vector<PositionTime> obs = {};
            for (auto& traj : trajectories.trajectories) {
                UAV drone = traj.conf().uav;
                for (size_t seg_id = 0; seg_id < traj.size(); seg_id++) {
                    const Segment3d& seg = traj[seg_id].maneuver;

                    double obs_time = traj.start_time(seg_id);
                    double obs_end_time = traj.end_time(seg_id);
                    TimeWindow seg_tw = TimeWindow{obs_time, obs_end_time};
                    if (tw.contains(seg_tw)) {
                        opt<std::vector<Cell>> opt_cells = segment_trace(seg, drone.view_depth, drone.view_width,
                                                                         firedata->ignitions);
                        if (opt_cells) {
                            for (const auto& c : *opt_cells) {
                                obs.emplace_back(
                                        PositionTime{firedata->ignitions.as_position(c), traj.start_time(seg_id)});
                            }
                        }
                    }
                }
            }
            return obs;
        }

        /*All the positions observed by the UAV camera*/
        vector<PositionTime> view_trace() const {
            return view_trace(time_window);
        }

        void insert_segment(size_t traj_id, const Segment3d& seg, size_t insert_loc, bool do_post_processing = true) {
            ASSERT(traj_id < trajectories.size());
            ASSERT(insert_loc <= trajectories[traj_id].size());
            trajectories[traj_id].insert_segment(seg, insert_loc);
            if (do_post_processing)
                post_process();
        }

        void erase_segment(size_t traj_id, size_t at_index, bool do_post_processing = true) {
            ASSERT(traj_id < trajectories.size());
            ASSERT(at_index < trajectories[traj_id].size());
            trajectories[traj_id].erase_segment(at_index);
            if (do_post_processing)
                post_process();
        }

        void replace_segment(size_t traj_id, size_t at_index, const Segment3d& by_segment) {
            replace_segment(traj_id, at_index, 1, std::vector<Segment3d>({by_segment}));
        }

        void
        replace_segment(size_t traj_id, size_t at_index, size_t n_replaced, const std::vector<Segment3d>& segments) {
            ASSERT(n_replaced > 0);
            ASSERT(traj_id < trajectories.size());
            ASSERT(at_index + n_replaced - 1 < trajectories[traj_id].size());

            // do not post process as we will do that at the end
            for (size_t i = 0; i < n_replaced; ++i) {
                erase_segment(traj_id, at_index, false);
            }

            for (size_t i = 0; i < segments.size(); ++i) {
                insert_segment(traj_id, segments.at(i), at_index + i, false);
            }

            post_process();
        }

        void post_process() {
            project_on_fire_front();
            smooth_trajectory();
        }

        /** Make sure every segment makes an observation, i.e., that the picture will be taken when the fire in traversing the main cell.
         *
         * If this is not the case for a given segment, its is projected on the firefront.
         * */
        void project_on_fire_front() {
            for (auto& traj : trajectories.trajectories) {
                size_t seg_id = traj.first_modifiable_maneuver();
                while (seg_id <= traj.last_modifiable_maneuver()) {
                    const Segment3d& seg = traj[seg_id].maneuver;
                    const double t = traj.start_time(seg_id);
                    opt<Segment3d> projected = firedata->project_on_firefront(seg, traj.conf().uav, t);
                    if (projected) {
                        if (*projected != seg) {
                            // original is different than projection, replace it
                            traj.replace_segment(seg_id, *projected);
                        }
                        seg_id++;
                    } else {
                        // segment has no projection, remove it
                        traj.erase_segment(seg_id);
                    }
                }
            }
        }

        /** Goes through all trajectories and erase segments causing very tight loops. */
        void smooth_trajectory() {
            for (auto& traj : trajectories.trajectories) {
                size_t seg_id = traj.first_modifiable_maneuver();
                while (seg_id < traj.last_modifiable_maneuver()) {
                    const Segment3d& current = traj[seg_id].maneuver;
                    const Segment3d& next = traj[seg_id + 1].maneuver;

                    const double euclidian_dist_to_next = current.end.as_point().dist(next.start.as_point());
                    const double dubins_dist_to_next = traj.conf().uav.travel_distance(current.end, next.start);

                    if (dubins_dist_to_next / euclidian_dist_to_next > 2.)
                        // tight loop, erase next and stay on this segment to check for tight loops on the new next.
                        traj.erase_segment(seg_id + 1);
                    else
                        // no loop detected, go to next
                        seg_id++;
                }
            }
        }

        /* Get the cells of the Raster
         * */
        template<typename GenRaster>
        static opt<std::vector<Cell>> segment_trace(const Segment3d& segment, double view_width,
                                                    double view_depth, const GenRaster& raster) {
            return SAOP::RasterMapper::segment_trace<GenRaster>(segment, view_width, view_depth, raster);
        }

    private:
        /** Constants used for computed the cost associated to a pair of points.
         * The cost is MAX_INDIVIDUAL_COST if the distance between two points
         * is >= MAX_INFORMATIVE_DISTANCE. It is be 0 if the distance is 0 and scales linearly between the two. */
        const double MAX_INFORMATIVE_DISTANCE = 500.;

        /** If a point is less than REDUNDANT_OBS_DIST aways from another observation, it useless to observe it.
         * This is defined such that those point are in the visible area when pictured. */
        const double REDUNDANT_OBS_DIST = 50.;
    };
}

#endif //PLANNING_CPP_PLAN_H
