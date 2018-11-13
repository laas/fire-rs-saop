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

#ifndef PLANNING_CPP_PYTHON_VNS_H
#define PLANNING_CPP_PYTHON_VNS_H

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

#include "core/dubinswind.hpp"
#include "core/trajectory.hpp"
#include "core/raster.hpp"
#include "cpp_py_utils.hpp"
#include "vns/factory.hpp"

#include "saop_logging.hpp"

namespace py = pybind11;

namespace SAOP {

    SearchResult
    plan_vns(std::string name, vector<TrajectoryConfig> configs, DRaster ignitions, DRaster elevation, const std::string& json_conf) {
        auto time = []() {
            struct timeval tp;
            gettimeofday(&tp, NULL);
            return (double) tp.tv_sec + ((double) (tp.tv_usec / 1000) / 1000.);
        };
        json conf = json::parse(json_conf);
        SAOP::check_field_is_present(conf, "min_time");
        const double min_time = conf["min_time"];
        SAOP::check_field_is_present(conf, "max_time");
        const double max_time = conf["max_time"];
        SAOP::check_field_is_present(conf, "save_every");
        const size_t save_every = conf["save_every"];
        SAOP::check_field_is_present(conf, "save_improvements");
        const bool save_improvements = conf["save_improvements"];
        SAOP::check_field_is_present(conf, "vns");
        SAOP::check_field_is_present(conf["vns"], "max_time");
        const size_t max_planning_time = conf["vns"]["max_time"];

        BOOST_LOG_TRIVIAL(debug) << "Pre-process fire data";
        double preprocessing_start = time();
        shared_ptr<FireData> fire_data = make_shared<FireData>(ignitions, elevation);
        double preprocessing_end = time();

        BOOST_LOG_TRIVIAL(debug) << "Build initial plan \"" << name << "\"";
        Plan p(name, configs, fire_data, TimeWindow{min_time, max_time});

        auto vns_dump = conf["vns"].dump();
        BOOST_LOG_TRIVIAL(debug) << "Using VNS search conf: " << vns_dump;
        auto vns = build_from_config(vns_dump);

        BOOST_LOG_TRIVIAL(info) << "Start planning \"" << p.name() << "\"";
        const double planning_start = time();
        auto res = vns->search(p, max_planning_time, save_every, save_improvements);
        const double planning_end = time();

        BOOST_LOG_TRIVIAL(info) << "Plan \"" << p.name() << "\" found in " << planning_end - planning_start << " seconds";
        BOOST_LOG_TRIVIAL(info) << "Plan \"" << p.name() << "\" utility: " << res.final().utility();
        BOOST_LOG_TRIVIAL(info) << "Plan \"" << p.name() << "\" duration: " << res.final().duration();
        res.metadata["planning_time"] = planning_end - planning_start;
        res.metadata["preprocessing_time"] = preprocessing_end - preprocessing_start;
        res.metadata["configuration"] = conf;
        return res;
    }

    SearchResult
    plan_vns(vector<TrajectoryConfig> configs, DRaster ignitions, DRaster elevation, const std::string& json_conf) {
       return plan_vns("unnamed", configs, ignitions, elevation, json_conf);
    }

    SearchResult
    replan_vns(SearchResult last_search, double after_time, DRaster ignitions, DRaster elevation,
               const std::string& json_conf) {
        auto time = []() {
            struct timeval tp;
            gettimeofday(&tp, NULL);
            return (double) tp.tv_sec + ((double) (tp.tv_usec / 1000) / 1000.);
        };
        json conf = json::parse(json_conf);
        SAOP::check_field_is_present(conf, "min_time");
//        const double min_time = conf["min_time"];
        SAOP::check_field_is_present(conf, "max_time");
//        const double max_time = conf["max_time"];
        SAOP::check_field_is_present(conf, "save_every");
        const size_t save_every = conf["save_every"];
        SAOP::check_field_is_present(conf, "save_improvements");
        const bool save_improvements = conf["save_improvements"];
        SAOP::check_field_is_present(conf, "vns");
        SAOP::check_field_is_present(conf["vns"], "max_time");
        const size_t max_planning_time = conf["vns"]["max_time"];

        BOOST_LOG_TRIVIAL(debug) << "Pre-process fire data";
        double preprocessing_start = time();
        shared_ptr<FireData> fire_data = make_shared<FireData>(ignitions, elevation);
        double preprocessing_end = time();

        BOOST_LOG_TRIVIAL(debug) << "Build and set a new plan from last search result ";
        Plan p = last_search.final();
        p.firedata(fire_data);
        p.freeze_before(after_time);
        p.project_on_fire_front();

        auto vns_dump = conf["vns"].dump();
        BOOST_LOG_TRIVIAL(debug) << "Using VNS search conf: " << vns_dump;
        auto vns = build_from_config(vns_dump);

        BOOST_LOG_TRIVIAL(info) << "Start planning";
        const double planning_start = time();
        auto res = vns->search(p, max_planning_time, save_every, save_improvements);
        const double planning_end = time();

        BOOST_LOG_TRIVIAL(info) << "Plan found in " << planning_end - planning_start << " seconds";
        BOOST_LOG_TRIVIAL(info) << "Best plan utility: " << res.final().utility();
        BOOST_LOG_TRIVIAL(info) << "Best plan duration: " << res.final().duration();
        res.metadata["planning_time"] = planning_end - planning_start;
        res.metadata["preprocessing_time"] = preprocessing_end - preprocessing_start;
        res.metadata["configuration"] = conf;

        return res;
    }
}


PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)

PYBIND11_MODULE(uav_planning, m) {
    m.doc() = "Python module for UAV trajectory planning";

#ifdef DEBUG
    srand(0);
    std::cerr << "Warning: Planning module compiled in debug mode. Expect slowness ;)\n";
#else
    srand(time(0));
#endif

    m.def("set_logger", [&m](py::object& logger) {
        SAOP::set_python_sink(logger);
    }, py::arg("logger").none(false), "Use a python logger as Boost::Log sink");

    py::class_<DRaster>(m, "DRaster")
            .def(py::init([](py::array_t<double, py::array::c_style | py::array::forcecast> arr,
                             double x_offset, double y_offset, double cell_width) {
                return new DRaster(as_vector<double>(arr), arr.shape(0), arr.shape(1), x_offset, y_offset, cell_width);
            }))
            .def("as_numpy", [](DRaster& self) {
                return as_nparray<double>(self.data, self.x_width, self.y_height);
            })
            .def_readonly("x_offset", &DRaster::x_offset)
            .def_readonly("y_offset", &DRaster::y_offset)
            .def_readonly("cell_width", &DRaster::cell_width);

    py::class_<LRaster>(m, "LRaster")
            .def(py::init([](py::array_t<long, py::array::c_style | py::array::forcecast> arr,
                             double x_offset, double y_offset, double cell_width) {
                return new LRaster(as_vector<long>(arr), arr.shape(0), arr.shape(1), x_offset, y_offset, cell_width);
            }))
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
            .def("contains", (bool (TimeWindow::*)(double time) const) &TimeWindow::contains,
                 py::arg("time"))
            .def("contains", (bool (TimeWindow::*)(const TimeWindow&) const) &TimeWindow::contains,
                 py::arg("time_window"))
            .def("__repr__",
                 [](const TimeWindow& tw) {
                     std::stringstream repr;
                     repr << "TimeWindow(" << tw.start << ", " << tw.end << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](TimeWindow& self) {
                return py::make_tuple(self.start, self.end);
            });

    py::class_<Cell>(m, "Cell")
            .def(py::init<const size_t, const size_t>(),
                 py::arg("x"), py::arg("y"))
            .def_readonly("x", &Cell::x)
            .def_readonly("y", &Cell::y)
            .def("__repr__",
                 [](const Cell& c) {
                     std::stringstream repr;
                     repr << "Cell(" << c.x << ", " << c.y << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Cell& self) {
                return py::make_tuple(self.x, self.y);
            });

    py::class_<Position>(m, "Position2d")
            .def(py::init<double, double>(),
                 py::arg("x"), py::arg("y"))
            .def_readonly("x", &Position::x)
            .def_readonly("y", &Position::y)
            .def("__repr__",
                 [](const Position& p) {
                     std::stringstream repr;
                     repr << "Position2d(" << p.x << ", " << p.y << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Position& self) {
                return py::make_tuple(self.x, self.y);
            });

    py::class_<Position3d>(m, "Position")
            .def(py::init<double, double, double>(),
                 py::arg("x"), py::arg("y"), py::arg("z"))
            .def_readonly("x", &Position3d::x)
            .def_readonly("y", &Position3d::y)
            .def_readonly("z", &Position3d::z)
            .def("__repr__",
                 [](const Position3d& p) {
                     std::stringstream repr;
                     repr << "Position(" << p.x << ", " << p.y << ", " << p.z << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Position3d& self) {
                return py::make_tuple(self.x, self.y, self.z);
            });

    py::class_<PositionTime>(m, "Position2dTime")
            .def(py::init<Position, double>(),
                 py::arg("point"), py::arg("time"))
            .def_readonly("pt", &PositionTime::pt)
            .def_readonly("time", &PositionTime::time)
            .def("__repr__",
                 [](const PositionTime& p) {
                     std::stringstream repr;
                     repr << "Position2dTime(" << p.pt.x << ", " << p.pt.y << ", " << p.time << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](PositionTime& self) {
                return py::make_tuple(py::make_tuple(self.pt.x, self.pt.y), self.time);
            });

    py::class_<Position3dTime>(m, "PositionTime")
            .def(py::init<Position3d, double>(),
                 py::arg("point"), py::arg("time"))
            .def_readonly("pt", &Position3dTime::pt)
            .def_readonly("time", &Position3dTime::time)
            .def("__repr__",
                 [](const Position3dTime& p) {
                     std::stringstream repr;
                     repr << "PositionTime(" << p.pt.x << ", " << p.pt.y << ", " << p.pt.z << ", " << p.time << ")";
                     return repr.str();
                 }
            )
            .def("as_tuple", [](Position3dTime& self) {
                return py::make_tuple(py::make_tuple(self.pt.x, self.pt.y, self.pt.z), self.time);
            });

    py::class_<TrajectoryManeuver>(m, "TrajectoryManeuver")
            .def(py::init<Segment3d, double, std::string>())
            .def_readwrite("maneuver", &TrajectoryManeuver::maneuver)
            .def_readwrite("time", &TrajectoryManeuver::time)
            .def_readwrite("name", &TrajectoryManeuver::name);

    py::class_<FireData, std::shared_ptr<FireData>>(m, "FireData")
            .def(py::init<const DRaster&, const DRaster&>(), py::arg("ignitions"), py::arg("elevation"))
            .def_readonly("ignitions", &FireData::ignitions)
            .def_readonly("traversal_end", &FireData::traversal_end)
            .def_readonly("propagation_directions", &FireData::propagation_directions)
            .def_readonly("elevation", &FireData::elevation);

    py::class_<Waypoint3d>(m, "Waypoint")
            .def(py::init<const double, const double, const double, const double>(),
                 py::arg("x"), py::arg("y"), py::arg("z"), py::arg("direction"))
            .def_readonly("x", &Waypoint3d::x)
            .def_readonly("y", &Waypoint3d::y)
            .def_readonly("z", &Waypoint3d::z)
            .def_readonly("dir", &Waypoint3d::dir)
            .def("__repr__", &Waypoint3d::to_string);

    py::class_<Waypoint>(m, "Waypoint2d")
            .def(py::init<const double, const double, const double>(),
                 py::arg("x"), py::arg("y"), py::arg("direction"))
            .def_readonly("x", &Waypoint::x)
            .def_readonly("y", &Waypoint::y)
            .def_readonly("dir", &Waypoint::dir)
            .def("__repr__", &Waypoint::to_string);

    py::class_<WindVector>(m, "WindVector")
            .def(py::init<const double, const double>(),
                 py::arg("x"), py::arg("y"))
            .def_property_readonly("x", &WindVector::x)
            .def_property_readonly("y", &WindVector::y)
            .def("dir", &WindVector::dir)
            .def("speed", &WindVector::modulo)
            .def("__repr__", &WindVector::to_string);

    py::class_<Segment3d>(m, "Segment")
            .def(py::init<const Waypoint3d, const double>())
            .def(py::init<const Waypoint3d, const Waypoint3d>())
            .def_readonly("start", &Segment3d::start)
            .def_readonly("end", &Segment3d::end)
            .def_readonly("length", &Segment3d::length)
            .def("__repr__", &Segment3d::to_string);

    py::class_<UAV>(m, "UAV")
            .def(py::init<std::string, const double, const double, const double>())
            .def_property_readonly("name", &UAV::name)
            .def_property_readonly("min_turn_radius", &UAV::min_turn_radius)
            .def_property_readonly("max_air_speed", &UAV::max_air_speed)
            .def_property_readonly("max_pitch_angle", &UAV::max_pitch_angle)
            .def("travel_distance", (double (UAV::*)(const Waypoint3d&, const Waypoint3d&) const)
                    &UAV::travel_distance, py::arg("origin"), py::arg("destination"))
            .def("travel_distance", (double (UAV::*)(const Waypoint&, const Waypoint&) const)
                    &UAV::travel_distance, py::arg("origin"), py::arg("destination"))
            .def("travel_time", (double (UAV::*)(const Waypoint3d&, const Waypoint3d&) const)
                    &UAV::travel_time, py::arg("origin"), py::arg("destination"))
            .def("travel_time", (double (UAV::*)(const Waypoint&, const Waypoint&) const)
                    &UAV::travel_time, py::arg("origin"), py::arg("destination"))
            .def("travel_time", (double (UAV::*)(const Waypoint3d&, const Waypoint3d&, const WindVector&) const)
                    &UAV::travel_time, py::arg("origin"), py::arg("destination"), py::arg("wind"))
            .def("travel_time", (double (UAV::*)(const Segment3d&, const WindVector&) const)
                    &UAV::travel_time, py::arg("segment"), py::arg("wind"))
            .def("path_sampling",
                 (std::vector<Waypoint3d> (UAV::*)(const Waypoint3d&, const Waypoint3d&, double) const)
                         &UAV::path_sampling, py::arg("origin"), py::arg("destination"), py::arg("step_size"))
            .def("path_sampling",
                 (std::vector<Waypoint3d> (UAV::*)(const Waypoint3d&, const Waypoint3d&, const WindVector&, double) const)
                         &UAV::path_sampling, py::arg("origin"), py::arg("destination"), py::arg("wind"), py::arg("step_size"))
            .def("path_sampling_airframe",
                 (std::vector<Waypoint3d> (UAV::*)(const Waypoint3d&, const Waypoint3d&, const WindVector&, double) const)
                         &UAV::path_sampling_airframe, py::arg("origin"), py::arg("destination"), py::arg("wind"),
                 py::arg("step_size"));

    py::class_<Trajectory>(m, "Trajectory")
            .def(py::init<const TrajectoryConfig&>())
            .def(py::init<TrajectoryConfig, const std::vector<TrajectoryManeuver>&>())
            .def_property_readonly("conf", &Trajectory::conf)
            .def("name", &Trajectory::name)
            .def("start_time", (double (Trajectory::*)() const) &Trajectory::start_time)
            .def("start_time", (double (Trajectory::*)(size_t) const) &Trajectory::start_time, py::arg("segment_index"))
            .def("end_time", (double (Trajectory::*)() const) &Trajectory::end_time)
            .def("end_time", (double (Trajectory::*)(size_t) const) &Trajectory::start_time, py::arg("segment_index"))
            .def_property_readonly("segments", &Trajectory::maneuvers)
            .def("segment", &Trajectory::maneuver, py::arg("index"))
            .def_property_readonly("maneuver_names", &Trajectory::maneuver_names)
//            .def_property_readonly("base_start", &Trajectory::base_start)
//            .def_property_readonly("base_end", &Trajectory::base_end)
            .def_property_readonly("start_times", &Trajectory::start_times)
            .def_property_readonly("modifiable", &Trajectory::modifiable)
            .def("can_modify", &Trajectory::can_modify, py::arg("maneuver_index"))
            .def("first_modifiable_id", &Trajectory::first_modifiable_maneuver)
            .def("slice", (Trajectory (Trajectory::*)(TimeWindow) const) &Trajectory::slice, py::arg("time_window"))
            .def("slice", [](Trajectory& self, py::tuple range) -> Trajectory {
                auto tw = TimeWindow(range[0].cast<double>(), range[1].cast<double>());
                return self.slice(tw);
            }, py::arg("time_window"))
            .def("slice", &Trajectory::slice, py::arg("time_window"))
            .def("length", &Trajectory::length)
            .def("__len__", &Trajectory::size)
            .def("duration", &Trajectory::duration)
            .def("as_waypoints", &Trajectory::as_waypoints)
            .def("sampled", &Trajectory::sampled, py::arg("step_size") = 1)
            .def("sampled_with_time", (std::pair<std::vector<Waypoint3d>, std::vector<double>> (Trajectory::*)(
                    double) const) &Trajectory::sampled_with_time, py::arg("step_size") = 1)
            .def("sampled_with_time", (std::pair<std::vector<Waypoint3d>, std::vector<double>> (Trajectory::*)
                         (TimeWindow, double) const) &Trajectory::sampled_with_time,
                 py::arg("time_range"), py::arg("step_size") = 1)
            .def("sampled_with_time", [](Trajectory& self, py::tuple time_range,
                                         double step) -> std::pair<std::vector<Waypoint3d>, std::vector<double>> {
                     return self.sampled_with_time(TimeWindow(
                             time_range[0].cast<double>(), time_range[1].cast<double>()), step);
                 },
                 py::arg("time_range"), py::arg("step_size") = 1)
            .def("with_waypoint_at_end", &Trajectory::with_waypoint_at_end)
            .def("__repr__", &Trajectory::to_string)
            .def("trace", [](Trajectory& self, const DRaster& r) {
                vector<PositionTime> trace = vector<PositionTime>{};
                for (auto i = 0ul; i <= self.size(); ++i) {
                    Plan::segment_trace(self[i].maneuver, self.conf().uav.view_width(), self.conf().uav.view_depth(),
                                        r);
                }
                return trace;
            }, py::arg("raster"));

    py::class_<TrajectoryConfig>(m, "TrajectoryConfig")
            .def(py::init<UAV, Waypoint3d, Waypoint3d, double, double>())
            .def(py::init<UAV, Waypoint3d, Waypoint3d, double, double, WindVector>())
            .def_readonly("uav", &TrajectoryConfig::uav)
            .def_readonly("max_flight_time", &TrajectoryConfig::max_flight_time)
            .def_static("build", [](UAV uav, double start_time, double max_flight_time) -> TrajectoryConfig {
                            return TrajectoryConfig(uav, start_time, max_flight_time);
                        }, "Constructor", py::arg("uav"), py::arg("start_time") = 0,
                        py::arg("max_flight_time") = std::numeric_limits<double>::max());

    py::class_<Plan>(m, "Plan")
            .def(py::init<std::string, std::vector<Trajectory>, shared_ptr<FireData>, TimeWindow, std::vector<PositionTime>>())
            /* TODO: Plan.trajectories() should be converted in an iterator instead of returning the internal trajectories vector*/
            .def("trajectories", [](Plan& self) { return Trajectories::get_internal_vector(self.trajectories()); })
            .def("name", &Plan::name)
            .def("utility", &Plan::utility)
            .def("utility_map", &Plan::utility_map)
            .def("duration", &Plan::duration)
            .def_property_readonly("firedata", (const FireData& (Plan::*)() const) &Plan::firedata)
            .def_readonly("time_window", &Plan::time_window)
            .def("observations", (vector<PositionTime> (Plan::*)() const) &Plan::observations)
            .def("observations", (vector<PositionTime> (Plan::*)(const TimeWindow&) const) &Plan::observations,
                 py::arg("tw"))
            .def("view_trace", (vector<PositionTime> (Plan::*)() const) &Plan::view_trace)
            .def("view_trace", (vector<PositionTime> (Plan::*)(const TimeWindow&) const) &Plan::view_trace,
                 py::arg("tw"));

    py::class_<SearchResult>(m, "SearchResult")
            .def("initial_plan", &SearchResult::initial)
            .def("final_plan", &SearchResult::final)
            .def_readonly("intermediate_plans", &SearchResult::intermediate_plans)
            .def("metadata", [](SearchResult& self) { return self.metadata.dump(); })
            .def("plan", [](SearchResult& self, size_t p) -> Plan {
                return self.intermediate_plans.at(p);
            })
            .def("plan", [](SearchResult& self, std::string p) -> Plan {
                if (p == "final") {
                    return self.final();
                } else if (p == "initial") {
                    return self.initial();
                } else {
                    throw std::invalid_argument("Wrong plan tag. Use 'initial', or 'final'");
                }
            });

    py::class_<DubinsWind>(m, "DubinsWind")
            .def(py::init<const Waypoint3d&, const Waypoint3d&, const WindVector&, double, double>(),
                 py::arg("from"), py::arg("to"), py::arg("wind"), py::arg("uav_airspeed"), py::arg("turn_radius"))
            .def("sampled", &DubinsWind::sampled, py::arg("l_step"))
            .def("sampled_airframe", &DubinsWind::sampled_airframe, py::arg("l_step"));

    m.def("replan_vns", SAOP::replan_vns, py::arg("last_search"), py::arg("after_time"), py::arg("ignitions_update"),
          py::arg("elevation_update"), py::arg("json_conf"), py::call_guard<py::gil_scoped_release>());


    m.def("plan_vns", (SearchResult (*)(vector<TrajectoryConfig>, DRaster, DRaster, const std::string&)) SAOP::plan_vns, py::arg("trajectory_configs"), py::arg("ignitions"), py::arg("elevation"),
          py::arg("json_conf"), py::call_guard<py::gil_scoped_release>());

    m.def("plan_vns", (SearchResult (*)(std::string, vector<TrajectoryConfig>, DRaster, DRaster, const std::string&)) SAOP::plan_vns, py::arg("plan_name"), py::arg("trajectory_configs"), py::arg("ignitions"), py::arg("elevation"),
          py::arg("json_conf"), py::call_guard<py::gil_scoped_release>());
}

#endif //PLANNING_CPP_PYTHON_VNS_H