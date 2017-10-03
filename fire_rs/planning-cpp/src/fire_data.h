#ifndef PLANNING_CPP_FIRE_DATA_H
#define PLANNING_CPP_FIRE_DATA_H

#include <algorithm>
#include <iostream>
#include <set>
#include "raster.h"
#include "waypoint.h"
#include "ext/optional.h"
#include "uav.h"
#include "utils.h"

using namespace std;


struct IsochroneCluster final {
    const TimeWindow time_window; /* Time span of this cluster. End time is excluded of the range: t ∈ [t_start, t_end)*/
    vector<Cell> cells; /* Cells belonging to this cluster */

    IsochroneCluster(const TimeWindow tw, vector<Cell> cell_vector) : time_window(tw), cells(cell_vector) {}

    /* IsochroneCluster "less than" comparison by center of time*/
    friend bool operator< (const IsochroneCluster& isochroneClusterA, const IsochroneCluster& isochroneClusterB){
        return isochroneClusterA.time_window.center() < isochroneClusterB.time_window.center();
    }
};

typedef set<shared_ptr<IsochroneCluster>, PComp<IsochroneCluster>> SetPIsochroneCluster_t;

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
    const DRaster elevation;

    /** Cells within the same timespan. */
    const vector<shared_ptr<IsochroneCluster>> isochrones;
    static constexpr double isochrone_timespan = 300.0;

    FireData(const DRaster &ignitions, const DRaster &elevation)
            : ignitions(ignitions),
              traversal_end(compute_traversal_ends(ignitions)),
              propagation_directions(compute_propagation_direction(ignitions)),
              elevation(elevation),
              isochrones(compute_isochrone_clusters(ignitions, traversal_end, isochrone_timespan)){}

    FireData(const FireData& from) = default;

    /** Returns true if the cell is eventually ignited. */
    bool eventually_ignited(Cell cell) const {
        return ignitions(cell) < numeric_limits<double>::max() /2;
    }

    /** Tries finding the closest cell on the firefront of the given time by going up or down the propagation slope.*/
    opt<Cell> project_on_fire_front(const Cell& cell, double time) const {
        ASSERT(ignitions.is_in(cell));
        if (time >= ignitions(cell) && time <= traversal_end(cell)) {
            return cell;
        } else {
            const double dir = positive_modulo(propagation_directions(cell), 2*M_PI);

            /** Make it discrete, a value of 0<= N <8 means the angle was rounded to N*PI/4 */
            long discrete_dir = lround(dir / (M_PI /4));
            ASSERT(discrete_dir >= 0 && discrete_dir <= 8)
            discrete_dir = discrete_dir % 8;  // 2PI == 0

            // compute relative coordinates of the cell in the main fire direction.
            int dx = -100;
            if(discrete_dir == 0 || discrete_dir == 1 || discrete_dir == 7)
                dx = 1;
            else if(discrete_dir == 3 || discrete_dir == 4 || discrete_dir == 5)
                dx = -1;
            else
                dx = 0;

            int dy = -100;
            if(discrete_dir == 1 || discrete_dir == 2 || discrete_dir == 3)
                dy = 1;
            else if(discrete_dir == 5 || discrete_dir == 6 || discrete_dir == 7)
                dy = -1;
            else
                dy = 0;

            ASSERT(dx >= -1 && dx <= 1);
            ASSERT(dy >= -1 && dy <= 1);

            Cell next_cell;
            if(time > traversal_end(cell)) {
                // move towards propagation direction
                next_cell = { cell.x+dx, cell.y + dy };
                if(!ignitions.is_in(next_cell) || ignitions(cell) > ignitions(next_cell))
                    // ignitions are not growing, we are in strange geometrical pattern inducing a local maximum, abandon
                    return {};
            } else {
                // move backwards the propagation direction
                ASSERT(time < ignitions(cell))
                next_cell = { cell.x-dx, cell.y-dy };
                if(!ignitions.is_in(next_cell) || ignitions(cell) < ignitions(next_cell))
                    // ignitions are not decreasing, we are in strange geometrical pattern inducing a local minimum, abandon
                    return {};
            }
            if(!ignitions.is_in(next_cell) || ! eventually_ignited(next_cell)) {
                return {};
            } else {
                return project_on_fire_front(next_cell, time);
            }
        }
    }

    /** Returns a segment whose visibility center is on cell on the firefront of the given time.
     *
     * This essentially projects a segment on the firefront, non-touching its orientation.
     *
     * The returned optional is empty if the projection failed.
     **/
    opt<Segment3d> project_on_firefront(const Segment3d& seg, const UAV& uav, double time) const {
        const Waypoint3d center = uav.visibility_center(seg);
        if(!ignitions.is_in(center))
            return {};
        const Cell cell = ignitions.as_cell(center);
        const opt<Cell> projected_cell = project_on_fire_front(cell, time);
        if(projected_cell)
            return uav.observation_segment(ignitions.x_coords(projected_cell->x), ignitions.y_coords(projected_cell->y),
                                           center.z, seg.start.dir, seg.length);
        else
            return {};
    }

private:
    /** Builds a raster containing the times at which the firefront leaves the cells. */
    static DRaster compute_traversal_ends(const DRaster &ignitions) {
        DRaster ie(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset, ignitions.cell_width);

        for(size_t x=0; x<ignitions.x_width; x++) {
            for(size_t y=0; y<ignitions.y_height; y++) {
                if(ignitions(x, y) < numeric_limits<double>::max() /2) {
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

    /** Computes local fire propagation direction. This is done by looking at the ignitions raster as an elevation raster
     * and finding main raising direction as it is done for computing slope.*/
    static DRaster compute_propagation_direction(const DRaster& ignitions) {
        DRaster pd(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset, ignitions.cell_width);
        /** Returns the ignitions time of (x+dx, y+dy). If it is out of the raster, or not ignited, it defaults to the ignition of (x,y).*/
        auto default_ignition = [ignitions](size_t x, size_t y, int dx, int dy) {
            const double def = ignitions(x, y);
            if(x == 0 && dx < 0) return def;
            if(x+dx >= ignitions.x_width) return def;
            if(y == 0 && dy < 0) return def;
            if(y+dy >= ignitions.y_height) return def;
            if(ignitions(x+dx, y+dy) >= numeric_limits<double>::max() /2) return def;
            return ignitions(x+dx, y+dy);
        };

        for(size_t x=0; x<ignitions.x_width; x++) {
            for(size_t y=0; y<ignitions.y_height; y++) {
                if(ignitions(x, y) < numeric_limits<double>::max() /2) {
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

    static vector<shared_ptr<IsochroneCluster>> compute_isochrone_clusters(const DRaster& ignitions,
                                                             const DRaster& traversal_end,
                                                             double cluster_timespan) {
        /* Isochrone clusters contain cells that burn in the cluster timespan.
         * Vector of IsochroneCluster should be in order.
         * NOT YET: Cluster timespans overlap 50% each other.
         *          This means that cluster_1 nominal timestamp is right in the border between cluster_0 and cluster_2.*/

        ASSERT(cluster_timespan > 0);

        vector<shared_ptr<IsochroneCluster>> isochrones;

        for(size_t x=0; x<ignitions.x_width; x++) {
            for(size_t y=0; y<ignitions.y_height; y++) {
                double ign_start = ignitions(x, y);
                ASSERT(ign_start > 0);
                double ign_end = traversal_end(x, y);
                ASSERT(ign_end > 0);

                Cell current_cell = Cell{x, y};

                bool isochrone_found = false;
                /*If eventually ignited*/
                if(abs(ign_start) < std::numeric_limits<double>::max() && abs(ign_end-ign_start) < std::numeric_limits<double>::max() ) {
                    TimeWindow cellTimeWindow = TimeWindow{ign_start, ign_end};
                    for (auto c : isochrones) {
                        if (c->time_window.contains(cellTimeWindow.center())) {
                            c->cells.push_back(current_cell);
                            isochrone_found = true;
                            break;
                        }
                    }

                    if (!isochrone_found) {
                        long range_index = static_cast<long>(cellTimeWindow.center() / cluster_timespan);
                        if (range_index <= 0) {
                            cout << "ERROR: range_index = " << range_index << " < 0." << endl;
                            cout << '\t' << "cellTimeWindow = " << cellTimeWindow << endl;
                        }
                        ASSERT(range_index >= 0);

                        double t_start = range_index*cluster_timespan;
                        double t_end = (range_index+1)*cluster_timespan;
                        shared_ptr<IsochroneCluster> new_isochrone = make_shared<IsochroneCluster>(
                                IsochroneCluster{TimeWindow{t_start, t_end}, vector<Cell>{current_cell}});
                        isochrones.push_back(new_isochrone);
                    }
                }
            }
        }

        /* Isochrone vector should be in order*/
        std::sort(isochrones.begin(), isochrones.end(),
                  [](std::shared_ptr<IsochroneCluster> a, std::shared_ptr<IsochroneCluster> b){
                      return a->time_window.center() < b->time_window.center();});

        /* Print the resulting isochrone vector */
//        for(auto iso : isochrones) {
//            cout << "Isochrone: " << '(' << iso->time_window.start/60 << ", " << iso->time_window.end/60 << ')' << endl;
//            for (auto c : iso->cells) {
//                cout << '\t' << c << endl;
//            }
//        }

        return isochrones;
    }
};

#endif //PLANNING_CPP_FIRE_DATA_H
