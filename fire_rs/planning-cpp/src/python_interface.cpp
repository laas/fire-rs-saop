#include <pybind11/pybind11.h>

#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays
#include "trajectory.h"
#include "raster.h"
#include "visibility.h"
#include "planning.h"

namespace py = pybind11;

/** Converts a nupy array to a vector */
template<class T>
std::vector<T> as_vector(py::array_t<T, py::array::c_style | py::array::forcecast> array) {
    std::vector<T> data(array.size());
    for(size_t x=0; x<array.shape(0); x++) {
        for(size_t y=0; y<array.shape(1); y++) {
            data[x + y*array.shape(0)] = *(array.data(x, y));
        }
    }
    return data;
}

/** Converts a vector to a 2D numpy array. */
template<class T>
py::array_t<T> as_nparray(std::vector<T> vec, size_t x_width, size_t y_height) {
    ASSERT(vec.size() == x_width * y_height)
    py::array_t<T, py::array::c_style | py::array::forcecast> array(std::vector<size_t> {x_width, y_height});
    for(size_t x=0; x<x_width; x++) {
        for (size_t y = 0; y < y_height; y++) {
            *(array.mutable_data(x, y)) = vec[x + y*x_width];
        }
    }
    return array;
}

PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

PYBIND11_PLUGIN(uav_planning) {
    py::module m("uav_planning", "Python module for AUV trajectory planning");

    srand(time(0));

    py::class_<DRaster>(m, "DRaster")
            .def("__init__", [](DRaster& self, py::array_t<double, py::array::c_style | py::array::forcecast> arr,
                                double x_offset, double y_offset, double cell_width)  {
                // create a new object and sibstitute to the given self
                new (&self) DRaster(as_vector<double>(arr), arr.shape(0), arr.shape(1), x_offset, y_offset, cell_width);
            })
            .def("as_numpy", [](DRaster& self) {
                return as_nparray<double>(self.data, self.x_width, self.y_height);
            })
            .def_readonly("x_offset", &DRaster::x_offset)
            .def_readonly("y_offset", &DRaster::y_offset)
            .def_readonly("cell_width", &DRaster::cell_width);

    py::class_<LRaster>(m, "LRaster")
            .def("__init__", [](LRaster& self, py::array_t<long, py::array::c_style | py::array::forcecast> arr,
                                double x_offset, double y_offset, double cell_width) {
                new (&self) LRaster(as_vector<long>(arr), arr.shape(0), arr.shape(1), x_offset, y_offset, cell_width);
            })
            .def("as_numpy", [](LRaster& self) {
                return as_nparray<long>(self.data, self.x_width, self.y_height);
            })
            .def_readonly("x_offset", &LRaster::x_offset)
            .def_readonly("y_offset", &LRaster::y_offset)
            .def_readonly("cell_width", &LRaster::cell_width);

    py::class_<TimeWindow>(m, "TimeWindow")
            .def(py::init<const double, const double>(),
                 py::arg("start"), py::arg("end"))
            .def_readonly("start", &TimeWindow::start)
            .def_readonly("end", &TimeWindow::end)
            .def("__repr__",
                 [](const TimeWindow &tw) {
                     std::stringstream repr;
                     repr << "TimeWindow(" << tw.start << ", " << tw.end << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](TimeWindow &self) {
                return py::make_tuple(self.start, self.end);
            } );

    py::class_<Cell>(m, "Cell")
            .def(py::init<const size_t, const size_t>(),
                 py::arg("x"), py::arg("y"))
            .def_readonly("x", &Cell::x)
            .def_readonly("y", &Cell::y)
            .def("__repr__",
                 [](const Cell &c) {
                     std::stringstream repr;
                     repr << "Cell(" << c.x << ", " << c.y << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Cell &self) {
                return py::make_tuple(self.x, self.y);
            } );

    py::class_<Point>(m, "Point")
            .def(py::init<double, double>(),
                 py::arg("x"), py::arg("y"))
            .def_readonly("x", &Point::x)
            .def_readonly("y", &Point::y)
            .def("__repr__",
                 [](const Point &p) {
                     std::stringstream repr;
                     repr << "Point(" << p.x << ", " << p.y << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Point &self) {
                return py::make_tuple(self.x, self.y);
            } );

    py::class_<PointTime>(m, "PointTime")
            .def(py::init<Point, double>(),
                 py::arg("point"), py::arg("time"))
            .def_readonly("pt", &PointTime::pt)
            .def_readonly("y", &PointTime::time)
            .def("__repr__",
                 [](const PointTime &p) {
                     std::stringstream repr;
                     repr << "PointTime(" << p.pt.x <<", " << p.pt.y << ", " << p.time << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](PointTime &self) {
                return py::make_tuple(py::make_tuple(self.pt.x, self.pt.y), self.time);
            } );

    py::class_<IsochroneCluster, shared_ptr<IsochroneCluster>>(m, "IsochroneCluster")
            .def("__init__", [](IsochroneCluster& self, const TimeWindow& tw, vector<Cell> cell_vector) {
                new (&self) IsochroneCluster(tw, cell_vector);
            })
            .def_readonly("time_window", &IsochroneCluster::time_window)
            .def_readonly("cells", &IsochroneCluster::cells);

    py::class_<FireData>(m, "FireData")
            .def("__init__", [](FireData& self, DRaster& ignitions) {
                new (&self) FireData(ignitions);
            })
            .def_readonly("ignitions", &FireData::ignitions)
            .def_readonly("traversal_end", &FireData::traversal_end)
            .def_readonly("propagation_directions", &FireData::propagation_directions)
//            .def_readonly_static("isochrone_timespan", &FireData::isochrone_timespan)
            .def_readonly("isochrones", &FireData::isochrones);

    py::class_<Waypoint>(m, "Waypoint")
            .def(py::init<const double, const double, const double>(),
                 py::arg("x"), py::arg("y"), py::arg("direction"))
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
            .def(py::init<const TrajectoryConfig&>())
            .def("start_time", (double (Trajectory::*)() const)&Trajectory::start_time)
            .def("end_time", (double (Trajectory::*)() const)&Trajectory::end_time)
            .def_readonly("segments", &Trajectory::traj)
            .def("length", &Trajectory::length)
            .def("duration", &Trajectory::duration)
            .def("as_waypoints", &Trajectory::as_waypoints, py::arg("step_size")= -1)
            .def("with_waypoint_at_end", &Trajectory::with_waypoint_at_end)
            .def("__repr__", &Trajectory::to_string);

    py::class_<Visibility>(m, "Visibility")
            .def(py::init<DRaster, double, double>())
            .def("set_time_window_of_interest", &Visibility::set_time_window_of_interest)
            .def("add_segment", &Visibility::add_segment)
            .def("remove_segment", &Visibility::remove_segment)
            .def("cost", &Visibility::cost)
            .def_readonly("visibility", &Visibility::visibility)
            .def_readonly("interest", &Visibility::interest);

    py::class_<TrajectoryConfig>(m, "TrajectoryConfig")
            .def(py::init<UAV, Waypoint, Waypoint, double, double>())
            .def_readonly("uav", &TrajectoryConfig::uav)
            .def_readonly("max_flight_time", &TrajectoryConfig::max_flight_time)
            .def_static("build", [](UAV uav, double start_time, double max_flight_time) -> TrajectoryConfig {
                return TrajectoryConfig(uav, start_time, max_flight_time);
            }, "Constructor", py::arg("uav"), py::arg("start_time") = 0,
                        py::arg("max_flight_time") = std::numeric_limits<double>::max());

    py::class_<Plan>(m, "Plan")
            .def_readonly("trajectories", &Plan::trajectories)
            .def("utility", &Plan::utility)
            .def("duration", &Plan::duration)
            .def_readonly("firedata", &Plan::firedata)
            .def("observations", &Plan::observations);

    py::class_<SearchResult>(m, "SearchResult")
            .def("initial_plan", &SearchResult::initial)
            .def("final_plan", &SearchResult::final)
            .def_readonly("intermediate_plans", &SearchResult::intermediate_plans)
            .def_readonly("planning_time", &SearchResult::planning_time)
            .def_readonly("preprocessing_time", &SearchResult::preprocessing_time);

    m.def("make_plan_vns", [](UAV uav, DRaster ignitions, double min_time, double max_time,
                              double max_flight_time, size_t save_every, bool save_improvements) -> SearchResult {
        auto fire_data = make_shared<FireData>(ignitions);
        TrajectoryConfig conf(uav, min_time, max_flight_time);
        Plan p(vector<TrajectoryConfig> { conf }, fire_data, TimeWindow{min_time, max_time});

        DefaultVnsSearch vns;

        auto res = vns.search(p, 0, save_every, save_improvements);
        return res;
    }, py::arg("uav"), py::arg("ignitions"), py::arg("min_time"), py::arg("max_time"), py::arg("max_flight_time"),
          py::arg("save_every") = 0, py::arg("save_improvements") = false);

    m.def("plan_vns", [](vector<TrajectoryConfig> configs, DRaster ignitions, double min_time, double max_time,
                         size_t save_every, bool save_improvements=false) -> SearchResult {
        auto time = []() {
            struct timeval tp;
            gettimeofday(&tp, NULL);
            return (double) tp.tv_sec + ((double)(tp.tv_usec / 1000) /1000.);
        };

        printf("Processing firedata data\n");
        double preprocessing_start = time();
        auto fire_data = make_shared<FireData>(ignitions);
        double preprocessing_end = time();

        printf("Building initial plan\n");
        Plan p(configs, fire_data, TimeWindow{min_time, max_time});

        printf("Planning\n");
        DefaultVnsSearch vns;
        const double planning_start = time();
        auto res = vns.search(p, 0, save_every, save_improvements);
        const double planning_end = time();
        printf("Plan found\n");
        res.planning_time = planning_end - planning_start;
        res.preprocessing_time = preprocessing_end - preprocessing_start;
        return res;
    }, py::arg("trajectory_configs"), py::arg("ignitions"), py::arg("min_time"), py::arg("max_time"),
          py::arg("save_every") = 0, py::arg("save_improvements") = false);

    return m.ptr();
}
