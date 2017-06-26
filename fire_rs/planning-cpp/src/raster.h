#ifndef PLANNING_CPP_RASTER_H
#define PLANNING_CPP_RASTER_H

#include <valarray>


struct Cell final {
    size_t x;
    size_t y;

    bool operator!=(Cell& pt) { return x != pt.x || y != pt.y; }

    friend std::ostream& operator<< (std::ostream& stream, const Cell& pt) {
        return stream << "(" << pt.x << ", " << pt.y <<")";
    }
};


struct Raster {
    std::vector<double> data;

    const size_t x_width;
    const size_t y_height;
    const double x_offset;
    const double y_offset;
    const double cell_width;

    Raster(std::vector<double> data, size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width) :
            data(data), x_width(x_width), y_height(y_height),
            x_offset(x_offset), y_offset(y_offset), cell_width(cell_width)
    {
        ASSERT(data.size() == x_width * y_height)
    }

    Raster(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width)
            : Raster(std::vector<double>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset, cell_width)
    {
    }

    void reset() {
        for(size_t i=0; i<x_width*y_height; i++)
            data[i] = 0;
    }

    inline double operator()(Cell cell) const {
        return (*this)(cell.x, cell.y);
    }
    inline double operator()(size_t x, size_t y) const {
        ASSERT(x >= 0 && x < x_width);
        ASSERT(y >= 0 && y <= y_height);
        return data[x + y*x_width];
    }

    inline void set(size_t x, size_t y, double value) {
        data[x + y*x_width] = value;
    }

    bool is_in(Cell cell) const {
        return cell.x < x_width && cell.y < y_height;
    }
    Point as_point(Cell cell) const {
        ASSERT(is_in(cell));
        return Point{ x_coords(cell.x), y_coords(cell.y) };
    }
    double x_coords(size_t x_index) const { return x_offset + cell_width * x_index; }
    double y_coords(size_t y_index) const { return y_offset + cell_width * y_index; }

    bool is_in(const Waypoint& wp) const {
        return is_x_in(wp.x) && is_y_in(wp.y);
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
    std::vector<long> data;

    const size_t x_width;
    const size_t y_height;

    const double x_offset;
    const double y_offset;
    const double cell_width;

    LRaster(std::vector<long> data, size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width) :
            data(data),
            x_width(x_width),
            y_height(y_height),
            x_offset(x_offset),
            y_offset(y_offset),
            cell_width(cell_width) {}

    LRaster(size_t x_width, size_t y_height, double x_offset, double y_offset, double cell_width)
            : LRaster(std::vector<long>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset, cell_width)
    {
    }

    inline long operator()(size_t x, size_t y) const {
        return data[x + y*x_width];
    }

    inline void set(size_t x, size_t y, long value) {
        data[x + y*x_width] = value;
    }

    void reset() {
        for(size_t i=0; i<x_width*y_height; i++)
            data[i] = 0;
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
