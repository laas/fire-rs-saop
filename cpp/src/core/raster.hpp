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

#ifndef PLANNING_CPP_RASTER_H
#define PLANNING_CPP_RASTER_H

#include <valarray>

#include "waypoint.hpp"
#include "../ext/optional.hpp"
#include "../ext/optional.hpp"

namespace SAOP {

    struct Cell final {
        size_t x;
        size_t y;

        Cell() = default;

        Cell(const size_t x, const size_t y) : x(x), y(y) {}

        bool operator!=(Cell& pt) const { return x != pt.x || y != pt.y; }

        bool operator==(const Cell& pt) const { return x == pt.x && y == pt.y; }

        Cell operator+(const Cell& pt) const { return Cell{x + pt.x, y + pt.y}; }

        Cell operator-(const Cell& pt) const { return Cell{x - pt.x, y - pt.y}; }

        friend std::ostream& operator<<(std::ostream& stream, const Cell& pt) {
            return stream << "(" << pt.x << ", " << pt.y << ")";
        }
    };

    template<typename T>
    struct GenRaster {
        std::vector<T> data;

        const size_t x_width;
        const size_t y_height;
        const double x_offset;
        const double y_offset;
        const double cell_width;

        GenRaster<T>(std::vector<T> data, size_t x_width, size_t y_height, double x_offset, double y_offset,
                     double cell_width) :
                data(data), x_width(x_width), y_height(y_height),
                x_offset(x_offset), y_offset(y_offset), cell_width(cell_width) {
            ASSERT(data.size() == x_width * y_height)
        }

        GenRaster<T>(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width)
                : GenRaster<T>(std::vector<T>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset,
                               cell_width) {}

        GenRaster<T>(const GenRaster<T>& like, T fill)
                : GenRaster<T>(std::vector<T>(like.x_width * like.y_height, fill), like.x_width, like.y_height,
                               like.x_offset,
                               like.y_offset, like.cell_width) {}

        void reset() {
            for (size_t i = 0; i < x_width * y_height; i++)
                data[i] = 0;
        }

        inline virtual T operator()(Cell cell) const {
            return (*this)(cell.x, cell.y);
        }

        inline virtual T operator()(size_t x, size_t y) const {
            ASSERT(x >= 0 && x < x_width);
            ASSERT(y >= 0 && y <= y_height);
            return data[x + y * x_width];
        }

        inline void set(size_t x, size_t y, T value) {
            data[x + y * x_width] = value;
        }

        inline void set(const Cell& c, T value) {
            set(c.x, c.y, value);
        }

        bool is_in(Cell cell) const {
            return cell.x < x_width && cell.y < y_height;
        }

        Position as_position(Cell cell) const {
            ASSERT(is_in(cell));
            return Position{x_coords(cell.x), y_coords(cell.y)};
        }

        double x_coords(size_t x_index) const {
            return x_offset + cell_width * x_index;
        }

        double y_coords(size_t y_index) const {
            return y_offset + cell_width * y_index;
        }

        bool is_in(const Waypoint& wp) const {
            return is_x_in(wp.x) && is_y_in(wp.y);
        }

        bool is_in(const Waypoint3d& wp) const {
            return is_x_in(wp.x) && is_y_in(wp.y);
        }

        bool is_in(const Position& pos) const {
            return is_x_in(pos.x) && is_y_in(pos.y);
        }

        bool is_in(const Position3d& pos) const {
            return is_x_in(pos.x) && is_y_in(pos.y);
        }

        bool is_x_in(double x_coord) const {
            return x_offset - cell_width / 2 <= x_coord && x_coord <= x_offset + cell_width * x_width - cell_width / 2;
        }

        bool is_y_in(double y_coord) const {
            return y_offset - cell_width / 2 <= y_coord && y_coord <= y_offset + cell_width * y_height - cell_width / 2;
        }

        Cell as_cell(const Waypoint& wp) const {
            ASSERT(is_in(wp));
            return Cell{x_index(wp.x), y_index(wp.y)};
        }

        Cell as_cell(const Waypoint3d& wp) const {
            ASSERT(is_in(wp));
            return Cell{x_index(wp.x), y_index(wp.y)};
        }

        Cell as_cell(const Position& pos) const {
            ASSERT(is_in(pos));
            return Cell{x_index(pos.x), y_index(pos.y)};
        }

        Cell as_cell(const Position3d& pos) const {
            ASSERT(is_in(pos));
            return Cell{x_index(pos.x), y_index(pos.y)};
        }

        size_t x_index(double x_coord) const {
            ASSERT(is_x_in(x_coord));
            return (size_t) lround((x_coord - x_offset) / cell_width);
        }

        size_t y_index(double y_coord) const {
            ASSERT(is_y_in(y_coord));
            return (size_t) lround((y_coord - y_offset) / cell_width);
        }
    };

    typedef GenRaster<double> DRaster;
    typedef GenRaster<long> LRaster;

    template<typename T>
    struct LocalRaster {
    public:

        std::vector<T> data;

        LocalRaster<T>(std::shared_ptr<GenRaster<T>> parent, std::vector<T> data, size_t x_width, size_t y_height,
                       Cell offset) : _parent(std::move(parent)), data(data), width(x_width), height(y_height),
                                      _offset(offset) {
            ASSERT(offset.x + x_width <= parent->x_width)
            ASSERT(offset.y + y_height <= parent->y_height)
            ASSERT(data.size() == x_width * y_height)
        }

        LocalRaster<T>(std::shared_ptr<GenRaster<T>> parent, size_t x_width, size_t y_height, Cell offset) :
                _parent(std::move(parent)), data(std::vector<T>(x_width * y_height, 0)), width(x_width),
                height(y_height), _offset(offset) {
            ASSERT(offset.x + x_width <= parent->x_width)
            ASSERT(offset.y + y_height <= parent->y_height)
        }

        /* Convert a Raster in a RasterUpdate*/
        LocalRaster<T>(GenRaster<T>&& raster, std::shared_ptr<GenRaster<T>> parent, size_t x_width, size_t y_height,
                       Cell offset) :
                _parent(std::move(parent)), data(std::move(raster.data)), width(x_width),
                height(y_height), _offset(offset) {
            ASSERT(offset.x + x_width <= parent->x_width)
            ASSERT(offset.y + y_height <= parent->y_height)
        }

        std::shared_ptr<GenRaster<T>> parent() const {
            return _parent;
        }

        Cell offset() const {
            return _offset;
        }

        void reset() {
            for (size_t i = 0; i < width * height; i++)
                data[i] = 0;
        }

        /* Get the data associated to a Cell in the ChildRaster refrence frame */
        inline virtual T operator()(Cell cell) const {
            return operator()(cell.x, cell.y);
        }

        /* Get the data associated to a Cell in the ChildRaster refrence frame */
        inline virtual T operator()(size_t x, size_t y) const {
            ASSERT(x >= 0 && x < width);
            ASSERT(y >= 0 && y <= height);
            return data[x + y * width];
        }

        void set(Cell cell, T value) {
            data[cell.x + cell.y * width] = value;
        }

        bool is_child_cell_in(Cell cell) const {
            return cell.x < width && cell.y < height;
        }

        bool is_parent_cell_in(Cell cell) const {
            return cell.x < width && cell.y < height;
        }

        Cell parent_cell(const Cell& child_raster_cell) const {
            return child_raster_cell + _offset;
        }

        Cell child_cell(const Cell& parent_raster_cell) const {
            return parent_raster_cell - _offset;
        }

        Position as_position(Cell cell) const {
            return _parent->as_position(parent_cell(cell));
        }

        bool is_in(const Waypoint& wp) const {
            return is_parent_cell_in(_parent->as_cell(wp));
        }

        bool is_in(const Waypoint3d& wp) const {
            return is_parent_cell_in(_parent->as_cell(wp));
        }

        bool is_in(const Position& pos) const {
            return is_parent_cell_in(_parent->as_cell(pos));
        }

        bool is_in(const Position3d& pos) const {
            return is_parent_cell_in(_parent->as_cell(pos));
        }

        Cell as_cell(const Waypoint& wp) const {
            ASSERT(is_in(wp));
            return child_cell(_parent->as_cell(wp));
        }

        Cell as_cell(const Waypoint3d& wp) const {
            ASSERT(is_in(wp));
            return child_cell(_parent->as_cell(wp));
        }

        Cell as_cell(const Position& pos) const {
            ASSERT(is_in(pos));
            return child_cell(_parent->as_cell(pos));
        }

        Cell as_cell(const Position3d& pos) const {
            ASSERT(is_in(pos));
            return child_cell(_parent->as_cell(pos));
        }

        bool applied() const {
            return _applied;
        }

        void apply_update() {
            ASSERT(!applied());

            auto parent_it = _parent->data.begin() + (_offset.x + _offset.y * _parent->x_width);
            auto update_it = data.begin();

            while (update_it <= data.end()) {
                std::copy(update_it, update_it + width, parent_it);
                std::advance(parent_it, _parent->x_width);
                std::advance(update_it, width);
            }
        }

    private:
        std::shared_ptr<GenRaster<T>> _parent;

        size_t width;
        size_t height;
        Cell _offset;

        bool _applied = false; // wether this update has been applyed or not
    };

    typedef LocalRaster<double> DLocalRaster;
    typedef LocalRaster<long> LLocalRaster;

    class RasterMapper {
    public:
        /* Get the cells resulting of mapping a maneuver into a GenRaster*/
        template<typename GenRaster>
        static opt<std::vector<Cell>> segment_trace(const Segment3d& segment, const double view_width,
                                                    const double view_depth, const GenRaster& raster) {
            std::vector<Cell> trace = {};

            // computes visibility rectangle
            // the rectangle is placed right in front of the plane. Its width is given by the view width of the UAV
            // (half of it on each side) and as length equal to the length of the segment + the view depth of the UAV
            const double w = view_width; // width of rect
            const double l = segment.length; // length of rect

            // coordinates of A, B and C corners, where AB and BC are perpendicular. D is the corner opposing A
            // UAV is at the center of AB
            const double ssx = segment.start.x - cos(segment.start.dir) * view_depth / 2;
            const double ssy = segment.start.y - sin(segment.start.dir) * view_depth / 2;

            const double ax = ssx + cos(segment.start.dir + M_PI / 2) * w / 2;
            const double ay = ssy + sin(segment.start.dir + M_PI / 2) * w / 2;
            const double bx = ssx - cos(segment.start.dir + M_PI / 2) * w / 2;
            const double by = ssy - sin(segment.start.dir + M_PI / 2) * w / 2;
            const double cx = ax + cos(segment.start.dir) * (l + view_depth);
            const double cy = ay + sin(segment.start.dir) * (l + view_depth);
            const double dx = bx + cos(segment.start.dir) * (l + view_depth);
            const double dy = by + sin(segment.start.dir) * (l + view_depth);

            // limits of the area in which to search for visible points
            // this is a subset of the raster that strictly contains the visibility rectangle
            const double min_x = std::max(std::min(std::min(ax, bx), std::min(cx, dx)) - raster.cell_width,
                                          raster.x_offset);
            const double max_x = std::min(std::max(std::max(ax, bx), std::max(cx, dx)) + raster.cell_width,
                                          raster.x_offset + raster.x_width * raster.cell_width - raster.cell_width / 2);
            const double min_y = std::max(std::min(std::min(ay, by), std::min(cy, dy)) - raster.cell_width,
                                          raster.y_offset);
            const double max_y = std::min(std::max(std::max(ay, by), std::max(cy, dy)) + raster.cell_width,
                                          raster.y_offset + raster.y_height * raster.cell_width -
                                          raster.cell_width / 2);

            // coordinates of where to start the search, centered on a cell
            const size_t start_x = raster.x_coords(raster.x_index(min_x));
            const size_t start_y = raster.y_coords(raster.y_index(min_y));

            // for each point possibly in the rectangle check if it is in the visible area and mark it as pending/visible when necessary
            for (double ix = start_x; ix <= max_x; ix += raster.cell_width) {
                for (double iy = start_y; iy <= max_y; iy += raster.cell_width) {
                    if (in_rectangle(ix, iy, ax, ay, bx, by, cx, cy)) {
                        // corresponding point in matrix coordinates
                        trace.push_back(Cell{raster.x_index(ix), raster.y_index(iy)});
                    }
                }
            }

            return trace;
        }

    private:
        /** Dot product of two vectors */
        static inline double dot(double x1, double y1, double x2, double y2) {
            return x1 * x2 + y1 * y2;
        }

        /** Returns true if the point (x,y) is in the rectangle defined by its two perpendicular sides AB and AC */
        static bool in_rectangle(double x, double y, double ax, double ay, double bx, double by, double cx, double cy) {
            const double dot_ab_am = dot(bx - ax, by - ay, x - ax, y - ay);
            const double dot_ab_ab = dot(bx - ax, by - ay, bx - ax, by - ay);
            const double dot_ac_am = dot(cx - ax, cy - ay, x - ax, y - ay);
            const double dot_ac_ac = dot(cx - ax, cy - ay, cx - ax, cy - ay);
            return 0 <= dot_ab_am && dot_ab_am <= dot_ab_ab &&
                   0 <= dot_ac_am && dot_ac_am <= dot_ac_ac;
        }
    };
}
#endif //PLANNING_CPP_RASTER_H
