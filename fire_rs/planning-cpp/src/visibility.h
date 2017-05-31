#ifndef PLANNING_CPP_VISIBILITY_H
#define PLANNING_CPP_VISIBILITY_H

#include "raster.h"
#include "trajectory.h"

using namespace std;


struct VisibilityMask : public LRaster {

    VisibilityMask(size_t width, size_t height, double x_offset, double y_offset, double cell_width)
            : LRaster(py::array_t<long>(vector<size_t> {width, height}), x_offset, y_offset, cell_width) {
        for(size_t x=0; x<width; x++) {
            for(size_t y=0; y<height; y++) {
                fast_data(x, y) = 0;
            }
        }
    }


    void add_visibility(const UAV& uav, const Segment segment) {
        const double w = uav.view_width;
        const double l = uav.view_depth + segment.length;
        const double ax = segment.start.x + cos(segment.start.dir + M_PI/2) * w/2;
        const double ay = segment.start.y + sin(segment.start.dir + M_PI/2) * w/2;
        const double bx = segment.start.x - cos(segment.start.dir + M_PI/2) * w/2;
        const double by = segment.start.y - sin(segment.start.dir + M_PI/2) * w/2;
        const double cx = ax + cos(segment.start.dir) * l;
        const double cy = ay + sin(segment.start.dir) * l;
        printf("Rect: m(%f, %f) (%f, %f) (%f, %f)\n", ax, ay, bx, by, cx, cy);

        const double greatest_side = max(w, l) + cell_width;
        const double center_x = x_coords(x_index(segment.start.x));
        const double center_y = y_coords(y_index(segment.start.y));

        // limits of the area in which to search for visible points
        const double min_x = max(center_x-greatest_side, x_offset);
        const double max_x = min(center_x+greatest_side, x_coords(x_width()));
        const double min_y = max(center_y-greatest_side, y_offset);
        const double max_y = min(center_y+greatest_side, y_coords(y_width()));

        printf("%f, %f, %f %f\n", min_x, max_x, min_y, max_y);

        for(double ix=min_x; ix<=max_x; ix+=cell_width) {
           for(double iy=min_y; iy<=max_y; iy+=cell_width) {
               if(in_rectangle(ix, iy, ax, ay, bx, by, cx, cy)) {
                   fast_data(x_index(ix), y_index(iy)) += 1;
               }
           }
        }

    }

private:
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
