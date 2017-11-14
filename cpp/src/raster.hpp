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

struct Cell final {
    size_t x;
    size_t y;

    Cell() = default;

    Cell(const size_t x, const size_t y) : x(x), y(y) {}

    bool operator!=(Cell& pt) { return x != pt.x || y != pt.y; }

    friend std::ostream& operator<< (std::ostream& stream, const Cell& pt) {
        return stream << "(" << pt.x << ", " << pt.y <<")";
    }
};

template <typename T>
struct GenRaster {
    std::vector<T> data;

    const size_t x_width;
    const size_t y_height;
    const double x_offset;
    const double y_offset;
    const double cell_width;

    GenRaster<T>(std::vector<T> data, size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width) :
            data(data), x_width(x_width), y_height(y_height),
            x_offset(x_offset), y_offset(y_offset), cell_width(cell_width)
    {
        ASSERT(data.size() == x_width * y_height)
    }

    GenRaster<T>(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width)
            : GenRaster<T>(std::vector<T>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset, cell_width) {}

    void reset() {
        for(size_t i=0; i<x_width*y_height; i++)
            data[i] = 0;
    }

    inline virtual T operator()(Cell cell) const {
        return (*this)(cell.x, cell.y);
    }

    inline virtual T operator()(size_t x, size_t y) const {
        ASSERT(x >= 0 && x < x_width);
        ASSERT(y >= 0 && y <= y_height);
        return data[x + y*x_width];
    }

    inline void set(size_t x, size_t y, T value) {
        data[x + y*x_width] = value;
    }

    bool is_in(Cell cell) const {
        return cell.x < x_width && cell.y < y_height;
    }

    Position as_position(Cell cell) const {
        ASSERT(is_in(cell));
        return Position{ x_coords(cell.x), y_coords(cell.y) };
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
        return x_offset-cell_width/2 <= x_coord && x_coord <= x_offset + cell_width * x_width-cell_width/2;
    }

    bool is_y_in(double y_coord) const {
        return y_offset-cell_width/2 <= y_coord && y_coord <= y_offset + cell_width * y_height-cell_width/2;
    }

    Cell as_cell(const Waypoint& wp) const {
        ASSERT(is_in(wp));
        return Cell{ x_index(wp.x), y_index(wp.y) };
    }

    Cell as_cell(const Waypoint3d& wp) const {
        ASSERT(is_in(wp));
        return Cell{ x_index(wp.x), y_index(wp.y) };
    }

    Cell as_cell(const Position& pos) const {
        ASSERT(is_in(pos));
        return Cell{ x_index(pos.x), y_index(pos.y) };
    }

    Cell as_cell(const Position3d& pos) const {
        ASSERT(is_in(pos));
        return Cell{ x_index(pos.x), y_index(pos.y) };
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

struct DiscreteDRaster : public DRaster {

    size_t interval = 100;

    DiscreteDRaster(std::vector<double> data, size_t x_width, size_t y_height, double x_offset, double y_offset,
                      double cell_width, size_t _interval) :
            DRaster(data, x_width, y_height, x_offset, y_offset, cell_width)
    {
        interval = _interval;
    }

    DiscreteDRaster(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width,
                    size_t _interval)
    : DRaster(std::vector<double>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset, cell_width)
    {
        interval = _interval;
    }

    DiscreteDRaster(DRaster raster, size_t _interval) : DRaster(std::move(raster)) {
        interval = _interval;
    }

    DiscreteDRaster(DRaster &&raster, size_t _interval) : DRaster(std::move(raster)) {
        interval = _interval;
    }

    inline double operator()(Cell cell) const override {
        return (*this)(cell.x, cell.y);
    }

    inline double operator()(size_t x, size_t y) const override {
        ASSERT(x >= 0 && x < x_width);
        ASSERT(y >= 0 && y <= y_height);
        double val = data[x + y*x_width];
        val = val + interval - fmod(val,interval); //FIXME: if interval is "1", does val stay the same after this operation?
        return val;
    }
};

#endif //PLANNING_CPP_RASTER_H
