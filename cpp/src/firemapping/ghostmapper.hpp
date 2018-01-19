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

#include "../core/fire_data.hpp"
#include "../core/raster.hpp"
#include "../core/trajectories.hpp"
#include "../core/trajectory.hpp"
#include "../core/uav.hpp"
#include "../ext/optional.hpp"

namespace SAOP {

    class GhostFireMapper {
        /* Create the map of the fire seen from a trajectory */


    public:
        explicit GhostFireMapper(shared_ptr<FireData> environment_gt) : _environment(environment_gt) {}

//        shared_ptr<FireData> environment_gt() const {
//            return _environment;
//        }

        /* Obtain a raster with only the observed fire cells set. The rest of the cells are Nan*/
        template<typename T>
        GenRaster<T> observed_fire(const Trajectory& traj) {
            GenRaster<T> fire = GenRaster<T>(_environment->ignitions, std::numeric_limits<T>::quiet_NaN());

            for (auto i = 0ul; i < traj.size(); ++i) {
                opt<std::vector<Cell>> ignited_cells =
                        *RasterMapper::segment_trace(traj.maneuver(i), traj.conf().uav.view_width,
                                                     traj.conf().uav.view_depth, fire);
                if (ignited_cells) {
                    for (const auto& c: *ignited_cells) {
                        if (traj.start_time(i) <= _environment->ignitions(c) &&
                            _environment->ignitions(c) <= traj.end_time(i)) {
                            fire.set(c, _environment->ignitions(c));
                        }
                    }
                }
            }
            return fire;
        }

        /* Obtain a raster with only the observed fire cells set. The rest of the cells are Nan*/
        template<typename T>
        GenRaster<T> observed_fire(const Trajectories& trajs) {
            GenRaster<T> fire = GenRaster<T>(_environment->ignitions, std::numeric_limits<T>::quiet_NaN());

            for (const auto& traj: trajs.trajectories) {
                for (auto i = 0ul; i < traj.size(); ++i) {
                    opt<std::vector<Cell>> ignited_cells =
                            *RasterMapper::segment_trace(traj.maneuver(i), traj.conf().uav.view_width,
                                                         traj.conf().uav.view_depth, fire);
                    if (ignited_cells) {
                        for (const auto& c: *ignited_cells) {
                            if (traj.start_time(i) <= _environment->ignitions(c) &&
                                _environment->ignitions(c) <= traj.end_time(i)) {
                                fire.set(c, _environment->ignitions(c));
                            }
                        }
                    }
                }
            }
            return fire;
        }

    private:
        shared_ptr<FireData> _environment;
    };

}
#define PLANNING_CPP_GHOSTMAPPER_HPP

#endif //PLANNING_CPP_GHOSTMAPPER_HPP
