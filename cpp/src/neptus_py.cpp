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

#ifndef PLANNING_CPP_PYTHON_NEPTUS_H
#define PLANNING_CPP_PYTHON_NEPTUS_H

#include <iomanip>
#include <sstream>

#include <pybind11/pybind11.h>
#include <pybind11/attr.h> // For py::arithmetic()
#include <pybind11/functional.h> // for std::function conversion
#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

#include "neptus/imc_comm.hpp"
#include "neptus/saop_server.hpp"

#include "saop_logging.hpp"

namespace py = pybind11;


PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)

PYBIND11_MODULE(neptus_interface, m) {
    m.doc() = "Python module for interfacing with neptus/dune software";

    m.def("set_logger", [&m](py::object& logger) {
        SAOP::set_python_sink(logger);
    }, py::arg("logger").none(false), "Use a python logger as Boost::Log sink");

    m.def("demo1_serialize_plan", [&m](SAOP::Trajectory& t, uint16_t uav_addr = 0x0c10,
                                       int projected_coordinate_system_epsg = SAOP::neptus::EPSG_ETRS89_LAEA) {
        std::string s = neptus::serialized_plan(t, "mission", uav_addr,
                                                projected_coordinate_system_epsg);
        return py::bytes(s);  // Return the data without transcoding

    }, py::arg("plan").none(false), py::arg("uav_addr"), py::arg("pcs"), "Get an IMC::PlanControl as bytes");

    m.def("demo1_check_ack", &neptus::check_ack, py::arg("msg").none(false));

    // Neptus interface
    py::class_<neptus::IMCComm, std::shared_ptr<neptus::IMCComm>>(m, "IMCComm")
            .def(py::init<unsigned short, std::string, std::string>(), py::arg("local_port"), py::arg("remote_ip"),
                 py::arg("remote_port"), "IMC communication with dune vehicles using UDP")
            .def(py::init<unsigned short>(), py::arg("remote_port"), "IMC communication with Neptus using TCP")
            .def("run", &neptus::IMCComm::run, py::call_guard<py::gil_scoped_release>());

    py::class_<neptus::GCS, std::shared_ptr<neptus::GCS >>(m, "GCS")
            .def(py::init<std::shared_ptr<neptus::IMCComm >>(), py::arg("imc"))
            .def(py::init<std::shared_ptr<neptus::IMCComm>,
                         std::function<void(neptus::TrajectoryExecutionReport)>,
                         std::function<void(neptus::UAVStateReport)>,
                         std::function<void(neptus::FireMapReport)>, int>(),
                 py::arg("imc"), py::arg("ter_cb"), py::arg("usr_cb"), py::arg("fmr_cb"), py::arg("pcs_epsg"),
                 py::call_guard<py::gil_scoped_release>())

            .def("start", (bool (neptus::GCS::*)(const Plan&, size_t, std::string, std::string)) &neptus::GCS::start,
                 py::arg("saop_plan"), py::arg("trajectory"), py::arg("plan_id"), py::arg("uav"),
                 py::call_guard<py::gil_scoped_release>())
            .def("start", (bool (neptus::GCS::*)(const Trajectory&, std::string)) &neptus::GCS::start,
                 py::arg("trajectory"), py::arg("uav"), py::call_guard<py::gil_scoped_release>())
            .def("start", (bool (neptus::GCS::*)(std::string, std::string)) &neptus::GCS::start,
                 py::arg("plan_id"), py::arg("uav"), py::call_guard<py::gil_scoped_release>())
            .def("loiter",
                 (bool (neptus::GCS::*)(std::string, LoiterManeuver, double, std::string)) &neptus::GCS::loiter,
                 py::arg("plan_id"), py::arg("loiter"), py::arg("speed"), py::arg("uav"),
                 py::call_guard<py::gil_scoped_release>())
            .def("load", (bool (neptus::GCS::*)(const Plan&, size_t, std::string, std::string)) &neptus::GCS::load,
                 py::arg("saop_plan"), py::arg("trajectory"), py::arg("plan_id"), py::arg("uav"),
                 py::call_guard<py::gil_scoped_release>())
            .def("stop", (bool (neptus::GCS::*)(std::string, std::string)) &neptus::GCS::stop,
                 py::arg("plan_id"), py::arg("uav"), py::call_guard<py::gil_scoped_release>())
            .def("set_wind", &neptus::GCS::set_wind, py::arg("speed"), py::arg("direction"), py::arg("uav"),
                 "Set the wind speed and direction (m/s, rad) for an uav")
            .def_property_readonly("available_vehicles", &neptus::GCS::available_vehicles)
            .def("is_ready", &neptus::GCS::is_ready);

    py::enum_<neptus::TrajectoryExecutionState>(m, "TrajectoryExecutionState", py::arithmetic())
            .value("Blocked", neptus::TrajectoryExecutionState::Blocked)
            .value("Ready", neptus::TrajectoryExecutionState::Ready)
            .value("Executing", neptus::TrajectoryExecutionState::Executing);

    py::enum_<neptus::TrajectoryExecutionOutcome>(m, "TrajectoryExecutionOutcome", py::arithmetic())
            .value("Nothing", neptus::TrajectoryExecutionOutcome::Nothing)
            .value("Success", neptus::TrajectoryExecutionOutcome::Success)
            .value("Failure", neptus::TrajectoryExecutionOutcome::Failure);

    py::class_<neptus::FireMapReport>(m, "FireMapReport")
            .def("__repr__", [](neptus::FireMapReport& self) -> std::string {
                std::stringstream ss;
                ss << self;
                return ss.str();
            })
            .def_readonly("timestamp", &neptus::FireMapReport::timestamp)
            .def_readonly("uav", &neptus::FireMapReport::uav)
            .def_readonly("firemap", &neptus::FireMapReport::firemap);

    py::class_<neptus::TrajectoryExecutionReport>(m, "TrajectoryExecutionReport")
            .def("__repr__", [](neptus::TrajectoryExecutionReport& self) -> std::string {
                std::stringstream ss;
                ss << self;
                return ss.str();
            })
            .def_readonly("timestamp", &neptus::TrajectoryExecutionReport::timestamp)
            .def_readonly("plan_id", &neptus::TrajectoryExecutionReport::plan_id)
            .def_readonly("uav", &neptus::TrajectoryExecutionReport::uav)
            .def_readonly("maneuver", &neptus::TrajectoryExecutionReport::maneuver)
            .def_readonly("maneuver_eta", &neptus::TrajectoryExecutionReport::maneuver_eta)
            .def_readonly("state", &neptus::TrajectoryExecutionReport::state)
            .def_readonly("last_outcome", &neptus::TrajectoryExecutionReport::last_outcome);

    py::class_<neptus::UAVStateReport>(m, "UAVStateReport")
            .def("__repr__", [](neptus::UAVStateReport& self) -> std::string {
                std::stringstream ss;
                ss << self;
                return ss.str();
            })
            .def_readonly("timestamp", &neptus::UAVStateReport::timestamp)
            .def_readonly("uav", &neptus::UAVStateReport::uav)
            .def_readonly("lat", &neptus::UAVStateReport::lat)
            .def_readonly("lon", &neptus::UAVStateReport::lon)
            .def_readonly("height", &neptus::UAVStateReport::height)
            .def_readonly("phi", &neptus::UAVStateReport::phi)
            .def_readonly("theta", &neptus::UAVStateReport::theta)
            .def_readonly("psi", &neptus::UAVStateReport::psi)
            .def_readonly("vx", &neptus::UAVStateReport::vx)
            .def_readonly("vy", &neptus::UAVStateReport::vy)
            .def_readonly("vz", &neptus::UAVStateReport::vz);
}

#endif //PLANNING_CPP_PYTHON_NEPTUS_H
