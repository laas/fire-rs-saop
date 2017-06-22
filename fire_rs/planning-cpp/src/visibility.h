#ifndef PLANNING_CPP_VISIBILITY_H
#define PLANNING_CPP_VISIBILITY_H

#include "raster.h"
#include "trajectory.h"

using namespace std;

struct Point final {
    size_t x;
    size_t y;

    constexpr bool operator!=(Point& pt) { return x != pt.x || y != pt.y; }

    friend std::ostream& operator<< (std::ostream& stream, const Point& pt) {
        return stream << "(" << pt.x << ", " << pt.y <<")";
    }
};

class Visibility {
public:
    const Raster ignitions;
    const double cell_width;
    LRaster visibility;
    LRaster interest;

    vector<Point> interesting_visited;
    vector<Point> interesting_pending;
    vector<double> pending_costs;

    Visibility(const Raster& ignitions, double time_window_min, double time_window_max)
            : ignitions(ignitions),
              cell_width(ignitions.cell_width),

              visibility(LRaster(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset, ignitions.cell_width)),
              interest(LRaster(ignitions.x_width, ignitions.y_height, ignitions.x_offset, ignitions.y_offset, ignitions.cell_width))
    {
        set_time_window_of_interest(time_window_min, time_window_max);
    }

    Visibility(const Visibility& from) = default;

    /** Mark as interesting all points whose ignition time lies between min and max.
     * All non-interesting points are not taken into account in the cost computation. */
    void set_time_window_of_interest(double min, double max) {
        reset();

        for(size_t x=0; x<ignitions.x_width; x++) {
            for(size_t y=0; y<ignitions.y_height; y++) {
                const double t = ignitions(x, y);
                if(min <= t && t <= max) {
                    interest.set(x, y, 1);
                    if(is_visible(x, y))
                        add_visited(Point{x, y});
                    else
                        add_pending(Point{x, y});
                }
            }
        }
    }


    inline bool is_of_interest(size_t x, size_t y) const { return interest(x, y) > 0; }
    inline bool is_visible(size_t x, size_t y) const { return visibility(x, y) > 0; }

    /** Mark the area covered by the segment as visible and update cost accordingly. */
    void add_segment(const UAV& uav, const Segment segment) {
        update_visibility(uav, segment, 1);
    }

    /** Remove the visibility contributed by the segment and update cost accordingly. */
    void remove_segment(const UAV& uav, const Segment segment) {
        update_visibility(uav, segment, -1);
    }

    double cost() const {
        double cost = 0.;
        for(size_t i=0; i<pending_costs.size(); i++) {
            cost += pending_costs[i];
        }
        return cost;
    }

    double cost_given_addition(const UAV& uav, Segment addition) {
        return cost_given(uav, vector<Segment> {addition}, vector<Segment> {});
    }
    double cost_given(const UAV& uav, const vector<Segment> additions, const vector<Segment> removal) {
        const double init_cost = cost();

        // apply changes
        for(auto it=additions.begin(); it != additions.end(); it++)
            add_segment(uav, *it);
        for(auto it=removal.begin(); it != removal.end(); it++)
            remove_segment(uav, *it);
        const double resulting_cost = cost();

        // rollback changes
        for(auto it=additions.begin(); it != additions.end(); it++)
            remove_segment(uav, *it);
        for(auto it=removal.begin(); it != removal.end(); it++)
            add_segment(uav, *it);

        assert(abs(init_cost - cost()) < 0.00001);
        return resulting_cost;
    }

private:
    /** Constants used for computed the cost associated to a pair of points.
     * The cost is MAX_INDIVIDUAL_COST if the distance between two points
     * is >= MAX_INFORMATIVE_DISTANCE. It is be 0 if the distance is 0 and scales linearly between the two. */
    const double MAX_INFORMATIVE_DISTANCE = 500.;
    const double MAX_INDIVIDUAL_COST = 1.;

    inline double cost_by_dist(const Point pt1, const Point pt2) const {
        const double dist = sqrt(pow((double) pt1.x - pt2.x, 2.) + pow((double) pt1.y - pt2.y, 2.)) * cell_width;
        const double cost = min(MAX_INFORMATIVE_DISTANCE, dist) / MAX_INFORMATIVE_DISTANCE * MAX_INDIVIDUAL_COST;
        assert(cost <= MAX_INDIVIDUAL_COST);
        return cost;
    }

    void reset() {
        interest.reset();
        interesting_pending.clear();
        interesting_visited.clear();
    }

    void add_visited(Point pt) {
        assert(visibility(pt.x, pt.y) == 1 && is_of_interest(pt.x, pt.y));
        interesting_visited.push_back(pt);

        // update costs of pending points if this new visited provides a better cost
        for(size_t i=0; i<interesting_pending.size(); i++) {
            const double cost_with_new = cost_by_dist(pt, interesting_pending[i]);
            if(cost_with_new < pending_costs[i])
                pending_costs[i] = cost_with_new;
        }
    }

    void remove_visited(Point pt) {
        assert(visibility(pt.x, pt.y) == 0 || !is_of_interest(pt.x, pt.y));

        // delete visited
        size_t index = 0;
        while(interesting_visited[index] != pt) {
            index++;
            assert(index < interesting_visited.size());  //Point is not in the visited list
        }
        interesting_visited.erase(interesting_visited.begin()+index);

        // update the cost of any pending whose cost was based on the presence of pt.
        for(size_t i=0; i<interesting_pending.size(); i++) {
            if(abs(pending_costs[i] - cost_by_dist(pt, interesting_pending[i])) < 0.0001) {
                // recompute cost of pending
                double best_cost = MAX_INDIVIDUAL_COST;
                for(auto it=interesting_visited.begin(); it!=interesting_visited.end(); it++) {
                    const double cost = cost_by_dist(interesting_pending[i], *it);
                    if(best_cost > cost) {
                        best_cost = cost;
                    }
                }
                pending_costs[i] = best_cost;
            }
        }
    }

    void add_pending(Point pt) {
        assert(visibility(pt.x, pt.y) == 0 && is_of_interest(pt.x, pt.y));
        interesting_pending.push_back(pt);

        // compute the cost of this point and store it
        double best_cost = MAX_INDIVIDUAL_COST;
        for(auto it=interesting_visited.begin(); it!=interesting_visited.end(); it++) {
            const double cost = cost_by_dist(pt, *it);
            if(best_cost > cost)
                best_cost = cost;
        }
        pending_costs.push_back(best_cost);
        assert(pending_costs.size() == interesting_pending.size());
    }

    void remove_pending(Point pt) {
        ASSERT(visibility(pt.x, pt.y) == 1 || !is_of_interest(pt.x, pt.y));
        size_t index = 0;
        while(interesting_pending[index] != pt) {
            index++;
            ASSERT(index < interesting_pending.size());  //Point is not in the pending list
        }
        interesting_pending.erase(interesting_pending.begin()+index);
        pending_costs.erase(pending_costs.begin()+index);
    }

    void update_visibility(const UAV& uav, const Segment segment, const int increment) {
        assert(increment == 1 || increment == -1);  // implementation currently only supports increment/decrement of 1

        // computes visibility rectangle
        // the rectangle is placed right in front of the plane. Its width is given by the view width of the UAV
        // (half of it on each side) and as length equal to the length of the segment + the view depth of the UAV
        const double w = uav.view_width; // width of rect
        const double l = uav.view_depth + segment.length; // length of rect

        // coordinates of A, B and C corners, where AB and BC are perpendicular. D is the corner opposing A
        // UAV is at the center of AB
        const double ax = segment.start.x + cos(segment.start.dir + M_PI/2) * w/2;
        const double ay = segment.start.y + sin(segment.start.dir + M_PI/2) * w/2;
        const double bx = segment.start.x - cos(segment.start.dir + M_PI/2) * w/2;
        const double by = segment.start.y - sin(segment.start.dir + M_PI/2) * w/2;
        const double cx = ax + cos(segment.start.dir) * l;
        const double cy = ay + sin(segment.start.dir) * l;
        const double dx = bx + cos(segment.start.dir) * l;
        const double dy = by + sin(segment.start.dir) * l;

        // limits of the area in which to search for visible points
        // this is a subset of the raster that strictly contains the visibility rectangle
        const double min_x = max(min(min(ax, bx), min(cx, dx)) - cell_width, ignitions.x_offset);
        const double max_x = min(max(max(ax, bx), max(cx, dx)) + cell_width, ignitions.x_offset + ignitions.x_width*cell_width-ignitions.cell_width/2);
        const double min_y = max(min(min(ay, by), min(cy, dy)) - cell_width, ignitions.y_offset);
        const double max_y = min(max(max(ay, by), max(cy, dy)) + cell_width, ignitions.y_offset + ignitions.y_height*cell_width -ignitions.cell_width/2);

        // coordinates of where to start the search, centered on a cell
        const double start_x = ignitions.x_coords(ignitions.x_index(min_x));
        const double start_y = ignitions.y_coords(ignitions.y_index(min_y));

        // for each point possibly in the rectangle check if it is in the visible area and mark it as pending/visible when necessary
        for(double ix=start_x; ix<=max_x; ix+=cell_width) {
           for(double iy=start_y; iy<=max_y; iy+=cell_width) {
               if(in_rectangle(ix, iy, ax, ay, bx, by, cx, cy)) {
                   // corresponding point in matrix coordinates
                   const Point pt{ ignitions.x_index(ix), ignitions.y_index(iy) };

                   visibility.set(pt.x, pt.y, visibility(pt.x, pt.y) + increment);
                   if(is_of_interest(pt.x, pt.y)) {
                       // pt is of interest
                       if (visibility(pt.x, pt.y) == 1 && increment == 1) {
                           // pt just became visited
                           remove_pending(pt);
                           add_visited(pt);
                       } else if(visibility(pt.x, pt.y) == 0 && increment == -1) {
                           // pt is not visited anymore
                           remove_visited(pt);
                           add_pending(pt);
                       }
                   }

               }
           }
        }

    }

    /** Dot product of two vectors */
    static inline double dot(double x1, double y1, double x2, double y2) {
        return x1 * x2 + y1 * y2;
    }
    /** Returns true if the point (x,y) is in the rectangle defined by its two perpendicular sides AB and AC */
    static bool in_rectangle(double x, double y, double ax, double ay, double bx, double by, double cx, double cy) {
        const double dot_ab_am = dot(bx-ax, by-ay, x-ax, y-ay);
        const double dot_ab_ab = dot(bx-ax, by-ay, bx-ax, by-ay);
        const double dot_ac_am = dot(cx-ax, cy-ay, x-ax, y-ay);
        const double dot_ac_ac = dot(cx-ax, cy-ay, cx-ax, cy-ay);
        return 0 <= dot_ab_am && dot_ab_am <= dot_ab_ab &&
               0 <= dot_ac_am && dot_ac_am <= dot_ac_ac;
    }

};

#endif //PLANNING_CPP_VISIBILITY_H
