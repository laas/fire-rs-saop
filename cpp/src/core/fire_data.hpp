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
#include <iostream>
#include <set>
#include "raster.hpp"
#include "waypoint.hpp"
#include "../ext/optional.hpp"
#include "uav.hpp"
#include "../utils.hpp"

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

        FireData(const DRaster& ignitions, const DRaster& elevation)
                : ignitions(ignitions),
                  traversal_end(compute_traversal_ends(ignitions)),
                  propagation_directions(compute_propagation_direction(ignitions)),
                  elevation(make_shared<DRaster>(elevation)) {}

        FireData(const FireData& from) = default;

        /** Returns true if the cell is eventually ignited. */
        bool eventually_ignited(Cell cell) const {
            return ignitions(cell) < numeric_limits<double>::max() / 2;
        }

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
        /** Builds a raster containing the times at which the firefront leaves the cells. */
        static DRaster compute_traversal_ends(const DRaster& ignitions);

        /** Computes local fire propagation direction. This is done by looking at the ignitions raster as an elevation raster
         * and finding main raising direction as it is done for computing slope.*/
        static DRaster compute_propagation_direction(const DRaster& ignitions);
    };
}
#endif //PLANNING_CPP_FIRE_DATA_H
