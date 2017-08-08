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
            : GenRaster<T>(std::vector<T>(x_width * y_height, 0), x_width, y_height, x_offset, y_offset, cell_width)
    {
    }

    void reset() {
        for(size_t i=0; i<x_width*y_height; i++)
            data[i] = 0;
    }

    inline T operator()(Cell cell) const {
        return (*this)(cell.x, cell.y);
    }
    inline T operator()(size_t x, size_t y) const {
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

//    std::vector<Point> points_around(Point center, long distance) const {
//        /*TODO: Test this function*/
//        std::vector<Point> circle_points;
//
//        /** Bresenham algorithm for circles http://members.chello.at/~easyfilter/bresenham.html*/
//        long delta_x = -distance;
//        long delta_y = 0;
//        long err = 2 - 2 * distance;
//        do{
//            if(is_x_in(center.x - delta_x) && is_y_in(center.y + delta_y)) {
//                circle_points.push_back(Point{center.x - delta_x, center.y + delta_y});
//            }
//            if(is_x_in(center.x - delta_y) && is_y_in(center.y - delta_x)) {
//                circle_points.push_back(Point{center.x - delta_y, center.y - delta_x});
//            }
//            if(is_x_in(center.x + delta_x) && is_y_in(center.y - delta_y)) {
//                circle_points.push_back(Point{center.x + delta_x, center.y - delta_y});
//            }
//            if(is_x_in(center.x + delta_y) && is_y_in(center.y + delta_x)) {
//                circle_points.push_back(Point{center.x + delta_y, center.y + delta_x});
//            }
//
//            distance = err;
//            if(distance <= delta_y) {
//                err += ++delta_y*2+1;
//            }
//            if (distance > delta_x || err > delta_y) {
//                err += ++delta_x*2+1;
//            }
//        }while (delta_x < 0);
//
//        return circle_points;
//    }
//
//    std::vector<Cell> cells_around(Cell center, long distance) const {
//        /*TODO: Test this function*/
//        std::vector<Cell> circle_cells;
//
//        /** Bresenham algorithm for circles http://members.chello.at/~easyfilter/bresenham.html*/
//        long delta_x = -distance;
//        long delta_y = 0;
//        long err = 2 - 2 * distance;
//        do{
//            if(is_x_in(center.x - delta_x) && is_y_in(center.y + delta_y)) {
//                circle_cells.push_back(Cell{center.x - delta_x, center.y + delta_y});
//            }
//            if(is_x_in(center.x - delta_y) && is_y_in(center.y - delta_x)) {
//                circle_cells.push_back(Cell{center.x - delta_y, center.y - delta_x});
//            }
//            if(is_x_in(center.x + delta_x) && is_y_in(center.y - delta_y)) {
//                circle_cells.push_back(Cell{center.x + delta_x, center.y - delta_y});
//            }
//            if(is_x_in(center.x + delta_y) && is_y_in(center.y + delta_x)) {
//                circle_cells.push_back(Cell{center.x + delta_y, center.y + delta_x});
//            }
//
//            distance = err;
//            if(distance <= delta_y) {
//                err += ++delta_y*2+1;
//            }
//            if (distance > delta_x || err > delta_y) {
//                err += ++delta_x*2+1;
//            }
//        }while (delta_x < 0);
//
//        return circle_cells;
//    }
};

typedef GenRaster<double> DRaster;
typedef GenRaster<long> LRaster;

#endif //PLANNING_CPP_RASTER_H
