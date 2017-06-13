#ifndef PLANNING_CPP_RASTER_H
#define PLANNING_CPP_RASTER_H

#include <pybind11/numpy.h>
namespace py = pybind11;


struct Raster {
    py::array_t<double> data;

    // for fast access, mutable keyword is required as the underlying object does not have const correctness
    mutable py::detail::unchecked_mutable_reference<double, 2> fast_data;
    const double x_offset;
    const double y_offset;
    const double cell_width;
    const size_t x_width = data.shape(0);
    const size_t y_height = data.shape(1);

    Raster(py::array_t<double, py::array::c_style | py::array::forcecast> data, double x_offset, double y_offset, double cell_width) :
            data(data), fast_data(this->data.mutable_unchecked<2>()),
            x_offset(x_offset), y_offset(y_offset), cell_width(cell_width) {}

    double operator()(size_t x, size_t y) const {
        return fast_data(x, y);
    }

    double x_coords(size_t x_index) const { return x_offset + cell_width * x_index; }
    double y_coords(size_t y_index) const { return y_offset + cell_width * y_index; }

    bool is_x_in(double x_coord) const {
        return x_offset-cell_width/2 <= x_coord && x_coord <= x_offset + cell_width * x_width;
    }
    bool is_y_in(double y_coord) const {
        return y_offset-cell_width/2 <= y_coord && y_coord <= y_offset + cell_width * y_height;
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

struct LRaster {
    py::array_t<long> data;

    // for fast access, mutable keyword is required as the underlying object does not have const correctness
    mutable py::detail::unchecked_mutable_reference<long, 2> fast_data;
    const double x_offset;
    const double y_offset;
    const double cell_width;
    const size_t x_width = data.shape(0);
    const size_t y_height = data.shape(1);

    LRaster(py::array_t<long, py::array::c_style | py::array::forcecast> data, double x_offset, double y_offset, double cell_width) :
            data(data),
            fast_data(this->data.mutable_unchecked<2>()),
            x_offset(x_offset),
            y_offset(y_offset),
            cell_width(cell_width) {}

    LRaster(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width)
            : data(py::array_t<long, py::array::c_style | py::array::forcecast>(vector<size_t> {x_width, y_height})),
              fast_data(this->data.mutable_unchecked<2>()),
              x_offset(x_offset),
              y_offset(y_offset),
              cell_width(cell_width)
    {
        reset();
    }

    LRaster(const LRaster& from) : LRaster(from.x_width, from.y_height, from.x_offset, from.y_offset, from.cell_width) {
        for(size_t x=0; x<x_width; x++)
            for(size_t y=0; y<y_height; y++)
                fast_data(x,y) = from.fast_data(x,y);
    }

    long operator()(size_t x, size_t y) const {
        return fast_data(x, y);
    }

    void reset() {
        for(size_t x=0; x<x_width; x++)
            for(size_t y=0; y<y_height; y++)
                fast_data(x, y) = 0;
    }

    double x_coords(size_t x_index) const { return x_offset + cell_width * x_index; }
    double y_coords(size_t y_index) const { return y_offset + cell_width * y_index; }

    bool is_x_in(double x_coord) const {
        return x_offset-cell_width/2 <= x_coord && x_coord <= x_offset + cell_width * x_width;
    }
    bool is_y_in(double y_coord) const {
        return y_offset-cell_width/2 <= y_coord && y_coord <= y_offset + cell_width * y_height;
    }

    size_t x_index(double x_coord) const {
        ASSERT(is_x_in(x_coord))
        return (size_t) lround((x_coord - x_offset) / cell_width);
    }

    size_t y_index(double y_coord) const {
        ASSERT(is_y_in(y_coord))
        return (size_t) lround((y_coord - y_offset) / cell_width);
    }
};



#endif //PLANNING_CPP_RASTER_H
