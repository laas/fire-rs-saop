#include <pybind11/pybind11.h>
#include <sstream>
#include "trajectory.h"

namespace py = pybind11;

PYBIND11_PLUGIN(uav_planning) {
    py::module m("uav_planning", "Generating primes in c++ with python bindings using pybind11");

    py::class_<Waypoint>(m, "Waypoint")
            .def(py::init<const double, const double, const double>())
            .def_readonly("x", &Waypoint::x)
            .def_readonly("y", &Waypoint::y)
            .def_readonly("dir", &Waypoint::dir)
            .def("__repr__", &Waypoint::to_string);

    py::class_<Segment>(m, "Segment")
            .def(py::init<const Waypoint, const double>())
            .def_readonly("start", &Segment::start)
            .def_readonly("end", &Segment::end)
            .def_readonly("length", &Segment::length);

    py::class_<UAV>(m, "UAV")
            .def(py::init<const double, const double>())
            .def_readonly("rho", &UAV::rho)
            .def_readonly("speed", &UAV::speed)
            .def("travel_distance", &UAV::travel_distance, py::arg("origin"), py::arg("destination"))
            .def("travel_time", &UAV::travel_time, py::arg("origin"), py::arg("destination"));





    return m.ptr();
}