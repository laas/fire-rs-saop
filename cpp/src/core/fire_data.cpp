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

#include "fire_data.hpp"

namespace SAOP {

    opt<Cell> FireData::project_on_fire_front(const Cell& cell, double time) const {
        ASSERT(ignitions.is_in(cell));
        Cell proj = project_closest_to_fire_front(cell, time);
        ASSERT(ignitions.is_in(proj));
        if (time >= ignitions(proj) && time <= traversal_end(proj)) {
            return proj;
        } else {
            return {};
        }
    }

    Cell FireData::project_closest_to_fire_front(const Cell& cell, double time) const {
        ASSERT(ignitions.is_in(cell));
        if (time >= ignitions(cell) && time <= traversal_end(cell)) {
            return cell;
        } else {
            const double dir = positive_modulo(propagation_directions(cell), 2 * M_PI);

            /** Make it discrete, a value of 0<= N <8 means the angle was rounded to N*PI/4 */
            long discrete_dir = lround(dir / (M_PI / 4));
            ASSERT(discrete_dir >= 0 && discrete_dir <= 8);
            discrete_dir = discrete_dir % 8;  // 2PI == 0

            // compute relative coordinates of the cell in the main fire direction.
            int dx = -100;
            if (discrete_dir == 0 || discrete_dir == 1 || discrete_dir == 7)
                dx = 1;
            else if (discrete_dir == 3 || discrete_dir == 4 || discrete_dir == 5)
                dx = -1;
            else
                dx = 0;

            int dy = -100;
            if (discrete_dir == 1 || discrete_dir == 2 || discrete_dir == 3)
                dy = 1;
            else if (discrete_dir == 5 || discrete_dir == 6 || discrete_dir == 7)
                dy = -1;
            else
                dy = 0;

            ASSERT(dx >= -1 && dx <= 1);
            ASSERT(dy >= -1 && dy <= 1);

            Cell next_cell;
            if (time > traversal_end(cell)) {
                // move towards propagation direction
                next_cell = {cell.x + dx, cell.y + dy};
                if (!ignitions.is_in(next_cell) || ignitions(cell) > ignitions(next_cell))
                    // ignitions are not growing, we are in strange geometrical pattern inducing a local maximum, abandon
                    return cell;
            } else {
                // move backwards the propagation direction
                ASSERT(time < ignitions(cell));
                next_cell = {cell.x - dx, cell.y - dy};
                if (!ignitions.is_in(next_cell) || ignitions(cell) < ignitions(next_cell))
                    // ignitions are not decreasing, we are in strange geometrical pattern inducing a local minimum, abandon
                    return cell;
            }
            if (!ignitions.is_in(next_cell) || !eventually_ignited(next_cell)) {
                return cell;
            } else {
                return project_closest_to_fire_front(next_cell, time);
            }
        }
    }

    opt<Segment3d> FireData::project_on_firefront(const Segment3d& seg, const UAV& uav, double time) const {
        const Waypoint3d center = uav.visibility_center(seg);
        if (!ignitions.is_in(center))
            return {};
        const Cell cell = ignitions.as_cell(center);
        const opt<Cell> projected_cell = project_on_fire_front(cell, time);
        if (projected_cell)
            return uav.observation_segment(ignitions.x_coords(projected_cell->x),
                                           ignitions.y_coords(projected_cell->y),
                                           center.z, seg.start.dir, seg.length);
        else
            return {};
    }

    Segment3d FireData::project_closest_to_fire_front(const Segment3d& seg, const UAV& uav, double time) const {
        const Waypoint3d center = uav.visibility_center(seg);
        if (!ignitions.is_in(center))
            return seg;
        const Cell cell = ignitions.as_cell(center);
        const opt<Cell> projected_cell = project_closest_to_fire_front(cell, time);
        if (projected_cell)
            return uav.observation_segment(ignitions.x_coords(projected_cell->x),
                                           ignitions.y_coords(projected_cell->y),
                                           center.z, seg.start.dir, seg.length);
        else
            return seg;
    }

    DRaster FireData::compute_traversal_ends(const DRaster& ignitions) {
        DRaster ie(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset,
                   ignitions.cell_width);

        for (size_t x = 0; x < ignitions.x_width; x++) {
            for (size_t y = 0; y < ignitions.y_height; y++) {
                if (ignitions(x, y) < numeric_limits<double>::max() / 2) {
                    // cell is ignited
                    // find the neighbor with highest ignition time
                    double max_neighbor = 0;
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            // exclude any neighbor that is of the grid or is never ignited.
                            if (dx == 0 && dy == 0) continue;
                            if (x == 0 && dx < 0) continue;
                            if (x + dx >= ignitions.x_width) continue;
                            if (y == 0 && dy < 0) continue;
                            if (y + dy >= ignitions.y_height) continue;
                            if (ignitions(x + dx, y + dy) >= numeric_limits<double>::max() / 2) continue;

                            max_neighbor = max(max_neighbor, ignitions(x + dx, y + dy));
                        }
                    }
                    if (max_neighbor <= ignitions(x, y))
                        ie.set(x, y, ignitions(x, y) + 180);  // propagation border, set traversal time to 3 minutes
                    else
                        ie.set(x, y, max_neighbor);
                } else {
                    // cell is never ignited, use same "infinite" value
                    ie.set(x, y, ignitions(x, y));
                }
            }
        }
        return ie;
    }

    DRaster FireData::compute_propagation_direction(const DRaster& ignitions) {
        DRaster pd(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset,
                   ignitions.cell_width);
        /** Returns the ignitions time of (x+dx, y+dy). If it is out of the raster, or not ignited, it defaults to the ignition of (x,y).*/
        auto default_ignition = [ignitions](size_t x, size_t y, int dx, int dy) {
            const double def = ignitions(x, y);
            if (x == 0 && dx < 0) return def;
            if (x + dx >= ignitions.x_width) return def;
            if (y == 0 && dy < 0) return def;
            if (y + dy >= ignitions.y_height) return def;
            if (ignitions(x + dx, y + dy) >= numeric_limits<double>::max() / 2) return def;
            return ignitions(x + dx, y + dy);
        };

        for (size_t x = 0; x < ignitions.x_width; x++) {
            for (size_t y = 0; y < ignitions.y_height; y++) {
                if (ignitions(x, y) < numeric_limits<double>::max() / 2) {
                    // cell is ignited, compute slope
                    auto ign = [x, y, default_ignition](int dx, int dy) { return default_ignition(x, y, dx, dy); };
                    const double prop_dx =
                            ign(1, -1) + 2 * ign(1, 0) + ign(1, 1) - ign(-1, -1) - 2 * ign(-1, 0) - ign(-1, 1);
                    const double prop_dy =
                            ign(1, 1) + 2 * ign(0, 1) + ign(-1, 1) - ign(1, -1) - 2 * ign(0, -1) - ign(-1, -1);
                    const double prop_dir = atan2(prop_dy, prop_dx);
                    pd.set(x, y, prop_dir);
                } else {
                    // cell is never ignited, set to default value
                    pd.set(x, y, 0);
                }
            }
        }
        return pd;
    }

}