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


    Plan::Plan(std::vector<TrajectoryConfig> traj_confs, std::shared_ptr<FireData> fire_data, TimeWindow tw,
               std::vector<PositionTime> observed_previously)
            : Plan("unnamed", Trajectories(traj_confs), fire_data, tw,
                   std::move(observed_previously), GenRaster<double>(fire_data->ignitions, 1.)) {}

    Plan::Plan(std::string name, std::vector<TrajectoryConfig> traj_confs, shared_ptr<FireData> fire_data,
               TimeWindow tw, std::vector<PositionTime> observed_previously)
            : Plan(name, Trajectories(traj_confs), fire_data, tw,
                   observed_previously, GenRaster<double>(fire_data->ignitions, 1.)) {}

    Plan::Plan(std::string name, std::vector<Trajectory> trajectories, std::shared_ptr<FireData> fire_data,
               TimeWindow tw, std::vector<PositionTime> observed_previously)
            : Plan(name, Trajectories(trajectories), fire_data, tw,
                   observed_previously, GenRaster<double>(fire_data->ignitions, 1.)) {}

    Plan::Plan(std::string name, std::vector<TrajectoryConfig> traj_confs, std::shared_ptr<FireData> fire_data,
               TimeWindow tw, GenRaster<double> utility) : Plan(name, Trajectories(traj_confs), fire_data, tw,
                                                                {}, std::move(utility)) {}

    Plan::Plan(std::string name, std::vector<Trajectory> trajectories, std::shared_ptr<FireData> fire_data,
               TimeWindow tw, GenRaster<double> utility): Plan(std::move(name), Trajectories(trajectories), fire_data, tw,
                                                               {}, std::move(utility)) {}

    Plan::Plan(std::string name, Trajectories trajectories, std::shared_ptr<FireData> fire_data, TimeWindow tw,
                   std::vector<PositionTime> observed_previously, GenRaster<double> utility)
            : time_window(tw), observed_previously(observed_previously), plan_name(name),
              trajs(trajectories), fire_data(fire_data), u_map(Utility(std::move(utility), fire_data)) {
        for (auto& t : trajectories) {
            ASSERT(t.conf().start_time >= time_window.start && t.conf().start_time <= time_window.end);
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

        u_map.reset(trajs);
    }

    json Plan::metadata() {
        json j;
        j["name"] = name();
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
        u_map.reset(trajs);
    }

    void Plan::erase_segment(size_t traj_id, size_t at_index, bool do_post_processing) {
        ASSERT(traj_id < trajs.size());
        ASSERT(at_index < trajs[traj_id].size());
        trajs[traj_id].erase_segment(at_index);
        if (do_post_processing) {
            post_process();
        }
        u_map.reset(trajs);
    }

    void Plan::replace_segment(size_t traj_id, size_t at_index, const Segment3d& by_segment) {
        replace_segment(traj_id, at_index, 1, std::vector<Segment3d>({by_segment}));
        u_map.reset(trajs);
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
        u_map.reset(trajs);
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
                        u_map.reset(trajs);
                    }
                    seg_id++;
                } else {
                    // segment has no projection, remove it
                    if (traj.can_modify(seg_id)) {
                        traj.erase_segment(seg_id);
                        u_map.reset(trajs);
                    } else {
                        seg_id++;
                    }
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
                    u_map.reset(trajs);
                } else {
                    // no loop detected, go to next
                    seg_id++;
                }
            }
        }
    }

    PReversibleTrajectoriesUpdate Plan::update(PReversibleTrajectoriesUpdate u, bool do_post_processing) {
//        std::cout << *u << std::endl;
        PReversibleTrajectoriesUpdate rev = u->apply(trajs);
        if (do_post_processing) {
            post_process();
        }
        u_map.reset(trajs);
        return rev;
    }

    void Plan::freeze_before(double time) {
        trajs.freeze_before(time);
    }

    void Plan::freeze_trajectory(std::string traj_name) {
        trajs.freeze_trajectory(traj_name);
    }


}