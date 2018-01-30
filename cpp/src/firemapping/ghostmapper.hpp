/* Copyright (c) 2018, CNRS-LAAS
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

#ifndef PLANNING_CPP_GHOSTMAPPER_HPP

#include <vector>
#include <memory>

#include "../core/fire_data.hpp"
#include "../core/raster.hpp"
#include "../core/trajectories.hpp"
#include "../core/trajectory.hpp"
#include "../core/uav.hpp"
#include "../ext/optional.hpp"

namespace SAOP {

    /* Create the map of the fire seen from a trajectory */
    template<typename T>
    class GhostFireMapper {
    public:
        explicit GhostFireMapper(shared_ptr<FireData> environment_gt)
                : _environment(environment_gt),
                  _fire_map(GenRaster<T>(environment_gt->ignitions, std::numeric_limits<T>::quiet_NaN())),
                  _observed(GenRaster<T>(environment_gt->ignitions, std::numeric_limits<T>::quiet_NaN())) {}

        GhostFireMapper(shared_ptr<FireData> environment_gt, GenRaster<T> firemap, GenRaster<T> observed)
                : _environment(std::move(environment_gt)),
                  _fire_map(std::move(firemap)),
                  _observed(std::move(observed)) {}

//        shared_ptr<FireData> environment_gt() const;

        /* Get the current fire map*/
        const GenRaster<T>& firemap() const {
            return _fire_map;
        }

        /* Get the current fire map*/
        const GenRaster<T>& observed() const {
            return _observed;
        }


        void observe(const Trajectory& traj) {
            auto wp_t = traj.as_waypoints_with_time();
            observed_fire(std::move(std::get<0>(wp_t)), std::move(std::get<1>(wp_t)), traj.conf().uav,
                          _fire_map, _observed);
        }

        void observe(const Trajectories& trajs) {
            for (const auto& t: trajs.trajectories) {
                observe(t);
            }
        }

        void observe(const vector<Waypoint3d>& wp_list, const vector<double>& time_list, const UAV& uav) {
            observed_fire(wp_list, time_list, uav, _fire_map, _observed);
        }

        GenRaster<T> observed_fire(const vector<Waypoint3d>& shot_wp_list,
                                                       const vector<double>& shot_time_list, const UAV& uav) const {
            GenRaster<T> fire = GenRaster<T>(_environment->ignitions, std::numeric_limits<T>::quiet_NaN());
            return observed_fire(shot_wp_list, shot_time_list, uav, _environment->ignitions);
        }

        GenRaster<T> observed_fire(const vector<Waypoint3d>& shot_wp_list,
                                                       const vector<double>& shot_time_list, const UAV& uav,
                                                       const GenRaster<T>& like) const {
//            ASSERT(shot_wp_list.size() > 1);
            ASSERT(shot_wp_list.size() == shot_time_list.size());
            ASSERT(like.is_like(_environment->ignitions))
            GenRaster<T> fire = GenRaster<T>(like.data, like.x_width, like.y_height,
                                             like.x_offset, like.y_offset, like.cell_width);

            auto it_wp = shot_wp_list.begin();
            auto it_t = shot_time_list.begin();
            for (; it_wp != shot_wp_list.end() - 1 || it_t != shot_time_list.end() - 1; ++it_wp, ++it_t) {
                if (!uav.is_turning(*it_wp, *(it_wp + 1))) {
                    opt<std::vector<Cell>> ignited_cells =
                            *RasterMapper::segment_trace(Segment3d{*it_wp, *(it_wp + 1)}, uav.view_width,
                                                         uav.view_depth, fire);
                    if (ignited_cells) {
                        for (const auto& c: *ignited_cells) {
                            TimeWindow fire_time_window = TimeWindow{_environment->ignitions(c),
                                                                     _environment->traversal_end(c)};
                            if (fire_time_window.contains(*it_t)) {
                                fire.set(c, _environment->ignitions(c));
                            }
                        }

                    }
                }
            }
            return fire;
        }

        void observed_fire(const vector<Waypoint3d>& shot_wp_list,
                                               const vector<double>& shot_time_list, const UAV& uav,
                                               GenRaster<T>& fire_raster, GenRaster<T>& obs_raster) const {
            ASSERT(shot_wp_list.size() > 1);
            ASSERT(shot_wp_list.size() == shot_time_list.size());
            ASSERT(fire_raster.is_like(_environment->ignitions))
            ASSERT(obs_raster.is_like(_environment->ignitions))

            auto it_wp = shot_wp_list.begin();
            auto it_t = shot_time_list.begin();
            for (; it_wp != shot_wp_list.end() - 1 || it_t != shot_time_list.end() - 1; ++it_wp, ++it_t) {
                if (!uav.is_turning(*it_wp, *(it_wp + 1))) {
                    opt<std::vector<Cell>> ignited_cells =
                            *RasterMapper::segment_trace(Segment3d{*it_wp, *(it_wp + 1)}, uav.view_width,
                                                         uav.view_depth, obs_raster);
                    if (ignited_cells) {
                        for (const auto& c: *ignited_cells) {
                            TimeWindow fire_time_window = TimeWindow{_environment->ignitions(c),
                                                                     _environment->traversal_end(c)};
                            obs_raster.set(c, *it_t); // Set the time the cell was observed
                            if (fire_time_window.contains(*it_t)) {
                                // Set the time the cell was observed ON FIRE
                                fire_raster.set(c, *it_t);
                            }
                        }

                    }
                }
            }
        }

        std::vector<PositionTime> observed_fire_locations(const vector<Waypoint3d>& shot_wp_list,
                                                                              const vector<double>& shot_time_list,
                                                                              const UAV& uav) const {
            ASSERT(shot_wp_list.size() == shot_time_list.size());
            std::vector<PositionTime> fire_cells = {};

            auto it_wp = shot_wp_list.begin();
            auto it_t = shot_time_list.begin();
            for (; it_wp != shot_wp_list.end() - 1 || it_t != shot_time_list.end() - 1; ++it_wp, ++it_t) {
                if (!uav.is_turning(*it_wp, *(it_wp + 1))) {
                    opt<std::vector<Cell>> ignited_cells =
                            *RasterMapper::segment_trace(Segment3d{*it_wp, *(it_wp + 1)}, uav.view_width,
                                                         uav.view_depth, _environment->ignitions);
                    if (ignited_cells) {
                        for (const auto& c: *ignited_cells) {
                            TimeWindow fire_time_window = TimeWindow{_environment->ignitions(c),
                                                                     _environment->traversal_end(c)};
                            if (fire_time_window.contains(*it_t)) {
                                fire_cells.emplace_back(PositionTime(_environment->ignitions.as_position(c),
                                                                     _environment->ignitions(c)));
                            }
                        }

                    }
                }
            }
            return fire_cells;
        }
    private:
        shared_ptr<FireData> _environment;
        GenRaster<T> _fire_map;
        GenRaster<T> _observed;
    };

}
#define PLANNING_CPP_GHOSTMAPPER_HPP

#endif //PLANNING_CPP_GHOSTMAPPER_HPP
