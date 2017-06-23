#ifndef PLANNING_CPP_WAYPOINT_H
#define PLANNING_CPP_WAYPOINT_H

#include <sstream>
#include "utils.h"

struct Waypoint final {
    // order and length is important for compatibility with dubins.cpp, allowing to simply cast a Waypoint* to a double*
    double x;
    double y;
    double dir;

    constexpr Waypoint(const Waypoint &wp) : x(wp.x), y(wp.y), dir(wp.dir) {}
    constexpr Waypoint(const double x, const double y, const double dir) : x(x), y(y), dir(dir) {};

    bool operator==(const Waypoint& o) const {
        return ALMOST_EQUAL(x,o.x) && ALMOST_EQUAL(y, o.y) && ALMOST_EQUAL(dir, o.dir);
    }

    std::string to_string() const {
        std::stringstream repr;
        repr << "(" << x<<", "<<y<<", "<<dir<<")";
        return repr.str();
    }

    Waypoint forward(double dist) const {
        const double new_x = x + cos(dir) * dist;
        const double new_y = y + sin(dir) * dist;
        return Waypoint(new_x, new_y, dir);
    }

    Waypoint rotate(double relative_angle) const {
        return Waypoint(x, y, dir+relative_angle);
    }

    Waypoint with_angle(double absolute_angle) const {
        return Waypoint(x, y, absolute_angle);
    }

    friend std::ostream& operator<< (std::ostream& stream, const Waypoint& w) {
        return stream << "(" << w.x << ", " << w.y << ", " << w.dir <<")";
    }
};

#endif //PLANNING_CPP_WAYPOINT_H
