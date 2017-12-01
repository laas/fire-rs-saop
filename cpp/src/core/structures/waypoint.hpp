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

#ifndef PLANNING_CPP_WAYPOINT_H
#define PLANNING_CPP_WAYPOINT_H

#include <cmath>
#include <sstream>
#include "../../utils.hpp"


struct Position final {
    double x;
    double y;

    Position() = default;

    constexpr Position(const double x, const double y) : x(x), y(y) {};

    double dist(const Position& pt) const {
        return sqrt(pow(x-pt.x, 2) + pow(y-pt.y, 2));
    }

    double angle_to(const Position& pt) const {
        return atan2(y-pt.y, x-pt.x);
    }
};

struct Position3d final {
    double x;
    double y;
    double z;

    Position3d() = default;

    constexpr explicit Position3d(const Position& pt) : x(pt.x), y(pt.y), z(0) {};

    constexpr Position3d(const double x, const double y, const double z) : x(x), y(y), z(z) {};

    constexpr Position3d(const Position& pt, const double z) : x(pt.x), y(pt.y), z(z) {};

    double dist(const Position3d& pt) const {
        return sqrt(pow(x-pt.x, 2) + pow(y-pt.y, 2) + pow(z-pt.z, 2));
    }

    double hor_dist(const Position3d& pt) const {
        return sqrt(pow(x-pt.x, 2) + pow(y-pt.y, 2));
    }

    double ver_angle_to(const Position3d& pt) const {
        return atan2(z-pt.z, hor_dist(pt));
    }

    double hor_angle_to(const Position3d& pt) const {
        return atan2(y-pt.y, x-pt.x);
    }

    Position as_2d() const {
        return Position{x, y};
    }
};

struct Waypoint final {
    // order and length is important for compatibility with dubins.cpp, allowing to simply cast a Waypoint* to a double*
    double x;
    double y;
    double dir;

    Waypoint() = default;

    constexpr Waypoint(const double x, const double y, const double dir) : x(x), y(y), dir(remainder(dir, 2 * M_PI)) {};

    bool operator==(const Waypoint& o) const {
        return ALMOST_EQUAL(x,o.x) && ALMOST_EQUAL(y, o.y) && ALMOST_EQUAL(dir, o.dir);
    }
    bool operator!=(const Waypoint& o) const {
        return !ALMOST_EQUAL(x,o.x) || !ALMOST_EQUAL(y, o.y) || !ALMOST_EQUAL(dir, o.dir);
    }

    std::string to_string() const {
        std::stringstream repr;
        repr << *this;
        return repr.str();
    }

    Waypoint forward(double dist) const {
        const double new_x = x + cos(dir) * dist;
        const double new_y = y + sin(dir) * dist;
        return Waypoint(new_x, new_y, dir);
    }

    Waypoint rotate(double relative_angle) const {
        return Waypoint(x, y, remainder(dir+relative_angle, 2 * M_PI));
    }

    Waypoint with_angle(double absolute_angle) const {
        return Waypoint(x, y, remainder(absolute_angle, 2 * M_PI));
    }

    friend std::ostream& operator<< (std::ostream& stream, const Waypoint& w) {
        return stream << "(" << w.x << ", " << w.y << ", " << w.dir <<")";
    }

    Position as_point() const {
        return Position{x, y};
    }
};

struct Waypoint3d final {
    double x;
    double y;
    double z;
    double dir;

    Waypoint3d() = default;

    constexpr Waypoint3d(const Waypoint3d &wp) : x(wp.x), y(wp.y), z(wp.z), dir(remainder(wp.dir, 2 * M_PI)) {}
    constexpr Waypoint3d(const double x, const double y, const double z, const double dir) : x(x), y(y), z(z), dir(remainder(dir, 2 * M_PI)) {};

    bool operator==(const Waypoint3d& o) const {
        return ALMOST_EQUAL(x,o.x) && ALMOST_EQUAL(y, o.y) && ALMOST_EQUAL(z, o.z) && ALMOST_EQUAL(dir, o.dir);
    }
    bool operator!=(const Waypoint3d& o) const {
        return !ALMOST_EQUAL(x,o.x) || !ALMOST_EQUAL(y, o.y) || !ALMOST_EQUAL(z, o.z) || !ALMOST_EQUAL(dir, o.dir);
    }

    std::string to_string() const {
        std::stringstream repr;
        repr << *this;
        return repr.str();
    }

    Waypoint3d forward(double dist) const {
        const double new_x = x + cos(dir) * dist;
        const double new_y = y + sin(dir) * dist;
        return Waypoint3d{new_x, new_y, z, dir};
    }

    Waypoint3d up(double dist) const {
        const double new_z = z + dist;
        return Waypoint3d{x, y, new_z, dir};
    }

    Waypoint3d rotate(double relative_angle) const {
        return Waypoint3d{x, y, z, dir+relative_angle};
    }

    Waypoint3d with_angle(double absolute_angle) const {
        return Waypoint3d{x, y, z, absolute_angle};
    }

    friend std::ostream& operator<< (std::ostream& stream, const Waypoint3d& w) {
        return stream << "(" << w.x << ", " << w.y << ", " << w.z << ", " << w.dir <<")";
    }

    Position3d as_point() const {
        return Position3d{x, y, z};
    }

    Waypoint as_2d() const {
        return Waypoint{x, y, dir};
    }
};


struct Segment final {
    Waypoint start;
    Waypoint end;
    double length;

    explicit Segment(const Waypoint& wp1) : start(wp1), end(wp1), length(0) {}
    Segment(const Waypoint& wp1, const Waypoint& wp2)
            : start(wp1), end(wp2), length(sqrt(pow(wp2.x-wp1.x, 2) + pow(wp2.y-wp1.y, 2))) {}
    Segment(const Waypoint& wp1, const double length)
            : start(wp1), end(Waypoint(wp1.x +cos(wp1.dir)*length, wp1.y + sin(wp1.dir)*length, wp1.dir)), length(length) {}
    Segment(const Position pt1, const Position pt2) : Segment(Waypoint {pt1.x, pt1.y, atan2(pt2.y - pt1.y, pt2.x - pt1.x)},
                                                        Waypoint {pt2.x, pt2.y, atan2(pt2.y - pt1.y, pt2.x - pt1.x)}) {}

    friend std::ostream& operator<< (std::ostream& stream, const Segment& s) {
        return stream << "<" << s.start << "--" << s.end << ", " << s.length << ">";
    }

    bool operator==(const Segment& o) const {
        return start == o.start && end == o.end && ALMOST_EQUAL(length, o.length);
    }

    bool operator!=(const Segment& o) const {
        return start != o.start || end != o.end || !ALMOST_EQUAL(length, o.length);
    }

    Segment reversed() const {
        return Segment{this->end.rotate(M_PI), this->start.rotate(M_PI)};
    }
};

struct Segment3d final {
    Waypoint3d start;
    Waypoint3d end;
    double length; // Full length of the segment
    double xy_length; // Length of the segment when projected into the xy plane

    explicit Segment3d(const Waypoint3d& wp1) : start(wp1), end(wp1), length(0), xy_length(0) {}

    Segment3d(const Waypoint3d& wp1, const Waypoint3d& wp2)
            : start(wp1), end(wp2), length(sqrt(pow(wp2.x-wp1.x, 2) + pow(wp2.y-wp1.y, 2) + pow(wp2.z-wp1.z, 2))),
              xy_length(sqrt(pow(wp2.x-wp1.x, 2) + pow(wp2.y-wp1.y, 2))) {}

    Segment3d(const Waypoint3d& wp1, double xy_length)
            : start(wp1), end(Waypoint3d(wp1.x + cos(wp1.dir) * xy_length, wp1.y + sin(wp1.dir) * xy_length, wp1.z, wp1.dir)),
              length(xy_length), xy_length(xy_length) {}

    Segment3d(const Position3d &pt1, const Position3d &pt2)
            : Segment3d(Waypoint3d {pt1.x, pt1.y, pt1.z, atan2(pt2.y - pt1.y, pt2.x - pt1.x)},
                        Waypoint3d {pt2.x, pt2.y, pt2.z, atan2(pt2.y - pt1.y, pt2.x - pt1.x)}) {
        ASSERT(ALMOST_EQUAL(start.z, end.z))
    }

    Segment3d(const Position &pt1, const Position &pt2, double z)
            : Segment3d(Waypoint3d {pt1.x, pt1.y, z, atan2(pt2.y - pt1.y, pt2.x - pt1.x)},
                        Waypoint3d {pt2.x, pt2.y, z, atan2(pt2.y - pt1.y, pt2.x - pt1.x)}) {
        ASSERT(ALMOST_EQUAL(start.z, end.z))
    }

    std::string to_string() const {
        std::stringstream repr;
        repr << *this;
        return repr.str();
    }

    friend std::ostream& operator<< (std::ostream& stream, const Segment3d& s) {
        return stream << "<" << s.start << "--" << s.end << ", " << s.length << ">";
    }

    bool operator==(const Segment3d& o) const {
        return start == o.start && end == o.end && ALMOST_EQUAL(length, o.length);
    }

    bool operator!=(const Segment3d& o) const {
        return start != o.start || end != o.end || !ALMOST_EQUAL(length, o.length);
    }

    Segment3d reversed() const {
        return Segment3d{this->end.rotate(M_PI), this->start.rotate(M_PI)};
    }

    Segment as_2d() const {
        return Segment{start.as_2d(), end.as_2d()};
    }
};

struct PositionTime final {
    Position pt;
    double time;

    PositionTime(const Position& p , const double t) : pt(p), time(t) {};
};

struct Position3dTime final {
    Position3d pt;
    double time;

    Position3dTime(const Position3d& p , const double t) : pt(p), time(t) {};
};

struct TimeWindow final {
    double start;
    double end;

    TimeWindow() = default;

    constexpr TimeWindow(const double start, const double end) : start(start < end ? start : end),
                                                                 end(start < end ? end : start) {}

    friend std::ostream& operator<<(std::ostream& os, const TimeWindow& time_window) {
        return os << "[" << time_window.start << ", " << time_window.end << ")";
    }

    double center() const {
        return (start+end)/2;
    }

    bool contains(double time) const {
        return start <= time && time < end;
    }

    bool contains(const TimeWindow &time_window) const {
        return start <= time_window.start && time_window.end < end;
    }

    bool intersects(const TimeWindow &time_window) const {
        return contains(time_window.start) || contains(time_window.end);
    }
};

struct PointTimeWindow final {
    Position pt;
    TimeWindow tw;
};

struct Point3dTimeWindow final {
    Position3d pt;
    TimeWindow tw;
};



#endif //PLANNING_CPP_WAYPOINT_H
