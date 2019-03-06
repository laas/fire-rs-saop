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
#include "utility.hpp"

double SAOP::Utility::utility() const {
    if (utility_cache && utility_map_cache) {
        return *utility_cache;
    }
    /* Recompute utility */
    GenRaster<double> utility = utility_map();
    auto accumulate_ignoring_nan = [](double a, double b) { return isnan(b) ? a : a + b; };
    utility_cache = std::accumulate(utility.begin(), utility.end(), 0., accumulate_ignoring_nan);
    return *utility_cache;

}

SAOP::GenRaster<double> SAOP::Utility::utility_map() const {
    if (utility_map_cache) {
        return *utility_map_cache;
    }
    utility_map_cache = utility_impl_trace();
    return *utility_map_cache;
}

SAOP::GenRaster<double> SAOP::Utility::initial_utility() const {
    return base_utility;
}

void Utility::reset(Trajectories trajs) {
    trajectories = std::move(trajs);
    utility_cache.reset();
    utility_map_cache.reset();
}

void Utility::reset(std::shared_ptr<FireData> firedata) {
    fire_data = std::move(firedata);
    utility_cache.reset();
    utility_map_cache.reset();
}

std::vector<std::pair<Position3dTime, Position3dTime>> Utility::straight_segments_of(const Trajectory& traj) {
    std::vector<std::pair<Position3dTime, Position3dTime>> segments = {};

    auto wp_time_pair = traj.sampled_with_time(50);

    if(wp_time_pair.first.size() < 2 || wp_time_pair.first.size() != wp_time_pair.second.size()) {
        return {};
    }

    std::vector<Waypoint3d>& wp_list = std::get<0>(wp_time_pair);
    std::vector<double>& time_list = std::get<1>(wp_time_pair);

    opt<Position3dTime> obs_segment_start = {};

    Waypoint3d prev_wp = *wp_list.begin();
    double prev_time = *time_list.begin();

    for (auto it = std::make_pair(wp_list.begin() + 1, time_list.begin() + 1);
         it.first != wp_list.end(); ++it.first, ++it.second) {
        if (!traj.conf().uav.is_turning(prev_wp, *it.first)) {
            if (!obs_segment_start) {
                obs_segment_start = Position3dTime(prev_wp.as_point(), prev_time);
            }
        } else {
            if (obs_segment_start) {
                segments.emplace_back(std::make_pair(*obs_segment_start,
                                                     Position3dTime(Position3dTime(prev_wp.as_point(), prev_time))));
                obs_segment_start.reset();
            }
        }

        prev_wp = *it.first;
        prev_time = *it.second;
    }

    return segments;
}

SAOP::GenRaster<double> SAOP::Utility::utility_impl_trace() const {
    auto u_map = base_utility;
    if (trajectories) {
        for (const auto& traj : *trajectories) {
            /* Identify straight portions of trajectory */
            auto straight_o = straight_segments_of(traj);
            for (const auto& o : straight_o) {
                Segment3d segment = Segment3d(o.first.pt, o.second.pt);
                TimeWindow segment_tw = TimeWindow(o.first.time, o.second.time);

                /*Search cells observed from the straight paths*/
                opt<std::vector<Cell>> trace = RasterMapper::segment_trace<GenRaster<double>>(segment,
                                                                                              traj.conf().uav.view_width(),
                                                                                              traj.conf().uav.view_depth(),
                                                                                              u_map);
                if (trace) {

                    for (const auto& c: *trace) {
                        TimeWindow fire_tw = TimeWindow(fire_data->ignitions(c), fire_data->traversal_end(c));
                        /*Extract utility from the observed cells*/
                        if (segment_tw.intersects(fire_tw) || segment_tw.contains(fire_tw) || fire_tw.contains(segment_tw)) {
                            u_map.set(c, MIN_UTILITY);
                        }
                    }
                }
            }

        }
    }
    return u_map;
}