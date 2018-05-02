/* Copyright (c) 2017-2018, CNRS-LAAS
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

#include "plan.hpp"

namespace SAOP {

    Plan::Plan(vector<TrajectoryConfig> traj_confs, shared_ptr<FireData> fdata, TimeWindow tw,
               vector<PositionTime> observed_previously)
            : time_window(tw), observed_previously(observed_previously),
              trajs(traj_confs), fire_data(std::move(fdata)) {
        for (auto& conf : traj_confs) {
            ASSERT(conf.start_time >= time_window.start && conf.start_time <= time_window.end);
        }

        std::vector<Cell> obs_prev_cells;
        obs_prev_cells.reserve(observed_previously.size());
        std::transform(observed_previously.begin(), observed_previously.end(),
                       std::back_inserter(obs_prev_cells),
                       [this](const PositionTime& pt) -> Cell { return this->firedata().ignitions.as_cell(pt.pt); });

        for (size_t x = 0; x < firedata().ignitions.x_width; x++) {
            for (size_t y = 0; y < firedata().ignitions.y_height; y++) {
                const double t = firedata().ignitions(x, y);
                if (time_window.start <= t && t <= time_window.end) {
                    Cell c{x, y};

                    // If the cell is in the observed_previously list, do not add it to possible_observations
                    if (std::find(obs_prev_cells.begin(), obs_prev_cells.end(), c)
                        == obs_prev_cells.end()) {
                        possible_observations.push_back(
                                PointTimeWindow{fire_data->ignitions.as_position(c),
                                                {fire_data->ignitions(c), fire_data->traversal_end(c)}});
                    }
                }
            }
        }

        utility_cache_invalid = true;
    }

    json Plan::metadata() {
        json j;
        j["duration"] = duration();
        j["utility"] = utility();
        j["num_segments"] = num_segments();
        j["trajectories"] = json::array();
        for (const Trajectory& t : trajs) {
            j["trajectories"].push_back(t);
        }
        return j;
    }

    vector<PositionTime> Plan::observations_full() const {
        std::vector<PositionTime> result = {};
        for (const auto& tr: trajs) {
            GhostFireMapper<double> gfm = GhostFireMapper<double>(fire_data);
            auto wp_and_t = tr.sampled_with_time(50);
            auto obs = gfm.observed_fire_locations(std::get<0>(wp_and_t), std::get<1>(wp_and_t), tr.conf().uav);
            result.insert(result.end(), obs.begin(), obs.end());
        }
        return result;
    }

    vector<PositionTime> Plan::observations(const TimeWindow& tw) const {
        vector<PositionTime> obs = std::vector<PositionTime>(observed_previously);
        for (const auto& traj : trajs) {
            UAV drone = traj.conf().uav;
            for (size_t seg_id = 0; seg_id < traj.size(); seg_id++) {
                const Segment3d& seg = traj[seg_id].maneuver;

                double obs_time = traj.start_time(seg_id);
                double obs_end_time = traj.end_time(seg_id);
                TimeWindow seg_tw = TimeWindow{obs_time, obs_end_time};
                if (tw.contains(seg_tw)) {
                    opt<std::vector<Cell>> opt_cells = segment_trace(seg, drone.view_depth(), drone.view_width(),
                                                                     fire_data->ignitions);
                    if (opt_cells) {
                        for (const auto& c : *opt_cells) {
                            if (fire_data->ignitions(c) <= obs_time && obs_time <= fire_data->traversal_end(c)) {
                                // If the cell is observable, add it to the observations list
                                obs.push_back(
                                        PositionTime{fire_data->ignitions.as_position(c), traj.start_time(seg_id)});
                            }
                        }
                    }
                }
            }
        }
        return obs;
    }

    /*All the positions observed by the UAV camera*/
    vector<PositionTime> Plan::view_trace(const TimeWindow& tw) const {
        vector<PositionTime> obs = {};
        for (const auto& traj : trajs) {
            UAV drone = traj.conf().uav;
            for (size_t seg_id = 0; seg_id < traj.size(); seg_id++) {
                const Segment3d& seg = traj[seg_id].maneuver;

                double obs_time = traj.start_time(seg_id);
                double obs_end_time = traj.end_time(seg_id);
                TimeWindow seg_tw = TimeWindow{obs_time, obs_end_time};
                if (tw.contains(seg_tw)) {
                    opt<std::vector<Cell>> opt_cells = segment_trace(seg, drone.view_depth(), drone.view_width(),
                                                                     fire_data->ignitions);
                    if (opt_cells) {
                        for (const auto& c : *opt_cells) {
                            obs.emplace_back(
                                    PositionTime{fire_data->ignitions.as_position(c), traj.start_time(seg_id)});
                        }
                    }
                }
            }
        }
        return obs;
    }

    void Plan::insert_segment(size_t traj_id, const Segment3d& seg, size_t insert_loc, bool do_post_processing) {
        ASSERT(traj_id < trajs.size());
        ASSERT(insert_loc <= trajs[traj_id].size());
        trajs[traj_id].insert_segment(seg, insert_loc);
        if (do_post_processing) {
            post_process();
        }
        utility_cache_invalid = true;
    }

    void Plan::erase_segment(size_t traj_id, size_t at_index, bool do_post_processing) {
        ASSERT(traj_id < trajs.size());
        ASSERT(at_index < trajs[traj_id].size());
        trajs[traj_id].erase_segment(at_index);
        if (do_post_processing) {
            post_process();
        }
        utility_cache_invalid = true;
    }

    void Plan::replace_segment(size_t traj_id, size_t at_index, const Segment3d& by_segment) {
        replace_segment(traj_id, at_index, 1, std::vector<Segment3d>({by_segment}));
        utility_cache_invalid = true;
    }

    void
    Plan::replace_segment(size_t traj_id, size_t at_index, size_t n_replaced, const std::vector<Segment3d>& segments) {
        ASSERT(n_replaced > 0);
        ASSERT(traj_id < trajs.size());
        ASSERT(at_index + n_replaced - 1 < trajs[traj_id].size());

        // do not post process as we will do that at the end
        for (size_t i = 0; i < n_replaced; ++i) {
            erase_segment(traj_id, at_index, false);
        }

        for (size_t i = 0; i < segments.size(); ++i) {
            insert_segment(traj_id, segments.at(i), at_index + i, false);
        }

        post_process();
        utility_cache_invalid = true;
    }

    void Plan::project_on_fire_front() {
        for (auto& traj : trajs) {
            size_t seg_id = traj.first_modifiable_maneuver();
            while (seg_id <= traj.last_modifiable_maneuver()) {
                const Segment3d& seg = traj[seg_id].maneuver;
                const double t = traj.start_time(seg_id);
                opt<Segment3d> projected = fire_data->project_on_firefront(seg, traj.conf().uav, t);
                if (projected) {
                    if (*projected != seg) {
                        // original is different than projection, replace it
                        traj.replace_segment(seg_id, *projected);
                        utility_cache_invalid = true;
                    }
                    seg_id++;
                } else {
                    // segment has no projection, remove it
                    traj.erase_segment(seg_id);
                    utility_cache_invalid = true;
                }
            }
        }
    }

    void Plan::smooth_trajectory() {
        for (auto& traj : trajs) {
            size_t seg_id = traj.first_modifiable_maneuver();
            while (seg_id < traj.last_modifiable_maneuver()) {
                const Segment3d& current = traj[seg_id].maneuver;
                const Segment3d& next = traj[seg_id + 1].maneuver;

                const double euclidian_dist_to_next = current.end.as_point().dist(next.start.as_point());
                const double dubins_dist_to_next = traj.conf().uav.travel_distance(current.end, next.start);

                if (dubins_dist_to_next / euclidian_dist_to_next > 2.) {
                    // tight loop, erase next and stay on this segment to check for tight loops on the new next.
                    traj.erase_segment(seg_id + 1);
                    utility_cache_invalid = true;
                } else {
                    // no loop detected, go to next
                    seg_id++;
                }
            }
        }
    }

    GenRaster<double> Plan::utility_comp_radial() const {
        GenRaster<double> u_map = GenRaster<double>(fire_data->ignitions, numeric_limits<double>::signaling_NaN());
        vector<PositionTime> done_obs = observations_full();
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
            u_map.set(u_map.as_cell(possible_obs.pt),
                      (max(sqrt(min_dist), REDUNDANT_OBS_DIST) - REDUNDANT_OBS_DIST) /
                      (MAX_INFORMATIVE_DISTANCE - REDUNDANT_OBS_DIST));
        }
        return u_map;
    }

    GenRaster<double> Plan::utility_comp_propagation() const {
        // Start with NaN utility everywhere
        GenRaster<double> u_map = GenRaster<double>(fire_data->ignitions, numeric_limits<double>::signaling_NaN());

        // Fill u_map observable cells with MAX_UTILITY
        for (auto& obs : possible_observations) {
            Cell obs_cell = u_map.as_cell(obs.pt);
            u_map.set(obs_cell, MAX_UTILITY);
        }

        // Set-up a priority queue
        auto time_cell_comp = [this](Cell a, Cell b) -> bool {
            // Use "less" to get the lowset ignition time on top of the queue
            return (*this).fire_data->ignitions(a) < (*this).fire_data->ignitions(b);
        };
        typedef std::priority_queue<Cell, vector<Cell>, decltype(time_cell_comp)> TimeCellPriorityQueue;
        auto prop_q = TimeCellPriorityQueue(time_cell_comp, vector<Cell>());

        // Utility map value is expressed in terms of utility to be recolted.
        // 0 meaning then that all the utility has been acquired by the UAVs

        // ugain_of_cell gives the utility value that has to be *substracted* from the U map.
        // Returns max_utility if fdc is the maximum front duration (slow moving front not very interesting)
        // and min_utility if fdc is the minimum front duration (fast moving front -> more interesing)
        auto u_of_cell = [this](const Cell& c, double max_utility, double min_utility) {
            double fdc = this->fire_data->front_duration(c);
            double Mfd = this->fire_data->max_front_duration();
            double mfd = this->fire_data->min_front_duration();

            if (!isnan(fdc)) {
                // fdc ~ mfd : u = MIN_UTILITY = 0
                // fdc ~ Mfd : u = MAX_UTILITY = 1
                return ((fdc - mfd) / (Mfd - mfd) * (max_utility - min_utility)) + min_utility;
            } else {
                return max_utility; // Let's say 1... There is no interest then on going there
            }
        };

        // Init u_map and propagation queue with observed cells
        for (const PositionTime& obs : observations_full()) {
            Cell obs_cell = u_map.as_cell(obs.pt);
            prop_q.push(obs_cell);

            auto a = u_of_cell(obs_cell, MAX_UTILITY, MIN_UTILITY);

            u_map.set(obs_cell, MIN_UTILITY);
        }

        double U_SPREAD_FACTOR = 1.;


        // Propagate utility using BFS
        // TODO: try DFS?
        while (!prop_q.empty()) {
            auto a_cell = prop_q.top();
            prop_q.pop();

            double a_u = 1 - (u_of_cell(a_cell, MAX_UTILITY - .1, MIN_UTILITY));
            double U_INC = U_SPREAD_FACTOR * a_u;
            auto neig_c = u_map.neighbor_cells(a_cell);
            for (auto& n_cell: neig_c) {
                // Discard not observable neighbors
                if (isnan(u_map(n_cell))) { continue; }
                // Do not go backwards in time
                if (fire_data->ignitions(n_cell) < fire_data->ignitions(a_cell)) { continue; }
                // Do not make utility worse in utility
                if (u_map(n_cell) <= u_map(a_cell) + U_INC) { continue; }

                // Apply utility degradation
                if (n_cell.x == a_cell.x || n_cell.y == a_cell.y) {
                    u_map.set(n_cell, u_map(a_cell) + U_INC);
                } else {
                    // If corner neighbor
                    u_map.set(n_cell, u_map(a_cell) + U_INC * M_SQRT1_2);
                }

                if (u_map(n_cell) < MAX_UTILITY) {
                    prop_q.push(n_cell);
                } else {
                    // Set utility to MAX_UTILITY and stop propagating from this cell
                    u_map.set(n_cell, MAX_UTILITY);
                }
            }
        }
        return u_map;
    }

    PReversibleTrajectoriesUpdate Plan::update(PReversibleTrajectoriesUpdate u, bool do_post_processing) {
//        std::cout << *u << std::endl;
        PReversibleTrajectoriesUpdate rev = u->apply(trajs);
        if (do_post_processing) {
            post_process();
        }
        utility_cache_invalid = true;
        return rev;
    }

    void Plan::freeze_before(double time) {
        trajs.freeze_before(time);
    }
}