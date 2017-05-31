#ifndef PLANNING_CPP_RASTER_H
#define PLANNING_CPP_RASTER_H

#include <pybind11/numpy.h>
namespace py = pybind11;


struct Raster {
    py::array_t<double> data;
    py::detail::unchecked_mutable_reference<double, 2> fast_data;
    double x_offset;
    double y_offset;
    double cell_width;

    Raster(py::array_t<double, py::array::c_style | py::array::forcecast> data, double x_offset, double y_offset, double cell_width) :
            data(data), fast_data(data.mutable_unchecked<2>()), x_offset(x_offset), y_offset(y_offset), cell_width(cell_width) {
    }

    double operator()(unsigned long x, unsigned long y) {
        return fast_data(x, y);
    }

    size_t x_width() { return data.shape(0); }
    size_t y_width() { return data.shape(1); }

    double x_coords(unsigned long x_index) { return x_offset + cell_width * x_index; }
    double y_coords(unsigned long y_index) { return y_offset + cell_width * y_index; }

    size_t x_index(double x_coord) { return (unsigned long) lround((x_coord - x_offset) / cell_width); }
    size_t y_index(double y_coord) { return (unsigned long) lround((y_coord - y_offset) / cell_width); }
};

struct LRaster {
    py::array_t<long> data;
    py::detail::unchecked_mutable_reference<long, 2> fast_data;
    double x_offset;
    double y_offset;
    double cell_width;

    LRaster(py::array_t<long, py::array::c_style | py::array::forcecast> data, double x_offset, double y_offset, double cell_width) :
            data(data), fast_data(data.mutable_unchecked<2>()), x_offset(x_offset), y_offset(y_offset), cell_width(cell_width) {
    }

    long operator()(size_t x, size_t y) {
        return fast_data(x, y);
    }

    size_t x_width() { return data.shape(0); }
    size_t y_width() { return data.shape(1); }

    double x_coords(size_t x_index) { return x_offset + cell_width * x_index; }
    double y_coords(size_t y_index) { return y_offset + cell_width * y_index; }

    size_t x_index(double x_coord) {
        assert(x_offset-cell_width/2 <= x_coord && x_coord <= x_offset + cell_width * x_width());
        return (size_t) lround((x_coord - x_offset) / cell_width);
    }

    size_t y_index(double y_coord) {
        assert(y_offset-cell_width/2 <= y_coord && y_coord <= y_offset + cell_width * y_width());
        return (size_t) lround((y_coord - y_offset) / cell_width);
    }
};



#endif //PLANNING_CPP_RASTER_H
