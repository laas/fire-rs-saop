#include <pybind11/pybind11.h>

// for conversions between c++ and python collections
#include <pybind11/stl.h>
#include <sstream>
#include "trajectory.h"
#include "local_search.h"

namespace py = pybind11;

PYBIND11_PLUGIN(uav_planning) {
    py::module m("uav_planning", "Generating primes in c++ with python bindings using pybind11");

    srand(time(NULL));

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
            .def_readonly("min_turn_radius", &UAV::min_turn_radius)
            .def_readonly("max_air_speed", &UAV::max_air_speed)
            .def("travel_distance", &UAV::travel_distance, py::arg("origin"), py::arg("destination"))
            .def("travel_time", &UAV::travel_time, py::arg("origin"), py::arg("destination"));

    py::class_<Trajectory>(m, "Trajectory") 
            .def(py::init<const UAV&>())
            .def_readonly("uav", &Trajectory::uav)
            .def("length", &Trajectory::length)
            .def("duration", &Trajectory::duration)
            .def("as_waypoints", &Trajectory::as_waypoints, py::arg("step_size")= -1)
            .def("with_waypoint_at_end", &Trajectory::with_waypoint_at_end)
            .def("__repr__", &Trajectory::to_string);

    m.def("improve", [](const Trajectory& traj) {
//        std::unique_ptr<Neighborhood<Trajectory>> neighborhood(new AlignTwoConsecutiveNeighborhood());
        auto n1 = std::make_shared<AlignTwoConsecutiveNeighborhood>();
        auto n2 = std::make_shared<OrientationChangeNeighborhood>();
        auto n3 = std::make_shared<TwoOrientationChangeNeighborhood>();
        auto n4 = std::make_shared<AlignOnNextNeighborhood>();
        auto n5 = std::make_shared<AlignOnPrevNeighborhood>();
        auto ns = std::make_shared<CombinedNeighborhood<Trajectory>>(
                std::vector<std::shared_ptr<Neighborhood<Trajectory>>> { n1, n2, n3, n4, n5 });
        Trajectory t_res = first_improvement_search(traj, *ns, 5000);
        return t_res;
    });

    return m.ptr();
}