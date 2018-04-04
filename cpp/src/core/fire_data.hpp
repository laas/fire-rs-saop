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

#ifndef PLANNING_CPP_FIRE_DATA_H
#define PLANNING_CPP_FIRE_DATA_H

#include <algorithm>
#include <functional>
#include <iostream>
#include <set>

#include "../ext/optional.hpp"
#include "../utils.hpp"
#include "raster.hpp"
#include "uav.hpp"
#include "waypoint.hpp"

using namespace std;

namespace SAOP {

    class FireData {
    public:
        /** Time at which the firefront reaches each cell.
         * MAX_DOUBLE, if it is never ignited (not burnable or propagation stopped early). */
        const DRaster ignitions;

        /** Time at which the firefront has entirely traversed each cell. */
        const DRaster traversal_end;

        /** Main fire propagation direction in each cell. */
        const DRaster propagation_directions;

        /* Terrain elevation */
        const shared_ptr<DRaster> elevation;

        FireData(const DRaster& ignition_raster, const DRaster& elevation_raster)
                : ignitions(ignition_raster),
                  traversal_end(compute_traversal_ends(ignition_raster)),
                  propagation_directions(compute_propagation_direction(ignition_raster)),
                  elevation(make_shared<DRaster>(elevation_raster)) {

            std::vector<double> durations(ignitions.data.size());
            // compute ignition duration
            std::transform(traversal_end.begin(), traversal_end.end(), ignitions.begin(), durations.begin(),
                           [](double a, double b) {
                               // Minus (ignoring non-ignited)
                               return b >= numeric_limits<double>::max() / 2 ?
                                      std::numeric_limits<double>::signaling_NaN() : a - b;
                           });
            // Find min and max durations
            std::pair<double, double> min_max = std::accumulate(
                    durations.begin(), durations.end(),
                    std::pair<double, double>({std::numeric_limits<double>::infinity(), .0}),
                    [](std::pair<double, double> a, double b) -> std::pair<double, double> {
                        // Accumulate min and max if b is eventually ignited (!NaN)
                        if (isnan(b)) {
                            return a;
                        } else {
                            return {std::min<double>(std::get<0>(a), b),
                                    std::max<double>(std::get<1>(a), b)};
                        }
                    });
            min_ign_duration = std::get<0>(min_max);
            max_ign_duration = std::get<1>(min_max);
        }

        FireData(const FireData& from) = default;

        /* Shortest fire front duration among all cells */
        double max_front_duration() const {
            return max_ign_duration;
        };

        /* Longest fire front duration among all cells */
        double min_front_duration() const {
            return min_ign_duration;
        };

        /* Duration of the fire front on Cell c. Returns NaN if the cell is never ignited. */
        double front_duration(const Cell& c) const {
            return eventually_ignited(c) ? traversal_end(c) - ignitions(c) : numeric_limits<double>::signaling_NaN();
        }

        /** Returns true if the cell is eventually ignited. */
        bool eventually_ignited(Cell cell) const {
            return ignitions(cell) < numeric_limits<double>::max() / 2;
        }

        /* Given a cell, get the next cell following the main propagation direction.*/
        opt<Cell> next_in_propagation_direction(const Cell& cell) const;

        /** Lookup a cell that ignited at `time`. Returns an empty option if no such cell was found. */
        opt<Cell> project_on_fire_front(const Cell& cell, double time) const;

        /** Finds the closest cell from the fire front of the given time by going up or down the propagation slope.*/
        Cell project_closest_to_fire_front(const Cell& cell, double time) const;

        /** Returns a segment whose visibility center is on cell on the firefront of the given time.
         *
         * This essentially projects a segment on the firefront, non-touching its orientation.
         *
         * The returned optional is empty if the projection failed.
         **/
        opt<Segment3d> project_on_firefront(const Segment3d& seg, const UAV& uav, double time) const;

        /** Returns a segment whose visibility center is on cell closest to the firefront of the given time.
         *
         * This essentially projects a segment as close as possible to the firefront, non-touching its orientation.
         **/
        Segment3d project_closest_to_fire_front(const Segment3d& seg, const UAV& uav, double time) const;

    private:
        double max_ign_duration;
        double min_ign_duration;

        /** Builds a raster containing the times at which the firefront leaves the cells. */
        static DRaster compute_traversal_ends(const DRaster& ignitions);

        /** Computes local fire propagation direction. This is done by looking at the ignitions raster as an elevation raster
         * and finding main raising direction as it is done for computing slope.*/
        static DRaster compute_propagation_direction(const DRaster& ignitions);
    };
}
#endif //PLANNING_CPP_FIRE_DATA_H
