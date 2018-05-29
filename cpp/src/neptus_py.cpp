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

#include <pybind11/pybind11.h>
#include <pybind11/attr.h> // For py::arithmetic()
#include <pybind11/functional.h> // for std::function conversion
#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

#include "neptus/saop_to_dune.hpp"
#include "neptus/imc_comm.hpp"
#include "neptus/saop_server.hpp"

namespace py = pybind11;


PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)

PYBIND11_MODULE(neptus_interface, m) {
    m.doc() = "Python module for interfacing with neptus/dune software";

    m.def("upload_plan", neptus::send_plan_to_dune, py::arg("ip"), py::arg("port"),
          py::arg("plan"), py::arg("plan_id"), py::arg("segment_extension"), py::arg("sampled"));

    m.def("start_plan", neptus::send_plan_start_request, py::arg("ip"), py::arg("port"),
          py::arg("plan_id"));

    m.def("set_wind", [](const std::string& ip, const std::string& port,
                         float direction, float speed) {
        neptus::DuneLink dl(ip, port);
        auto wsm = neptus::WindSpeedFactory::make_message(direction, speed);
        dl.send(wsm);
    }, py::arg("ip"), py::arg("port"), py::arg("direction"), py::arg("speed"));

    py::class_<neptus::DuneLink>(m, "DuneLink")
            .def(py::init<std::string, std::string>(), py::arg("ip"), py::arg("port"));

    py::class_<neptus::IMCComm,
            std::shared_ptr<neptus::IMCComm>>(m, "IMCComm")
            .def(py::init())
            .def("run", &neptus::IMCComm::run, py::call_guard<py::gil_scoped_release>());

    py::enum_<neptus::GCSCommandOutcome>(m, "GCSCommandOutcome", py::arithmetic())
            .value("Unknown", neptus::GCSCommandOutcome::Unknown)
            .value("Success", neptus::GCSCommandOutcome::Success)
            .value("Failure", neptus::GCSCommandOutcome::Failure);

    py::class_<neptus::GCS,
            std::shared_ptr<neptus::GCS>>(m, "GCS")
            .def(py::init<std::shared_ptr<neptus::IMCComm>>(), py::arg("imc"))
            .def(py::init<std::shared_ptr<neptus::IMCComm>,
                         const std::function<void(neptus::PlanExecutionReport)>,
                         const std::function<void(neptus::UAVStateReport)>>(),
                 py::arg("imc"), py::arg("per_cb"), py::arg("usr_cb"), py::call_guard<py::gil_scoped_release>())
            .def("start", (neptus::GCSCommandOutcome (neptus::GCS::*)(const Plan&)) &neptus::GCS::start,
                 py::arg("plan"), py::call_guard<py::gil_scoped_release>())
            .def("start", (neptus::GCSCommandOutcome (neptus::GCS::*)()) &neptus::GCS::start, py::call_guard<py::gil_scoped_release>())
            .def("load", (neptus::GCSCommandOutcome (neptus::GCS::*)(const Plan&)) &neptus::GCS::load,
                 py::arg("plan"), py::call_guard<py::gil_scoped_release>())
            .def("stop", &neptus::GCS::stop, py::call_guard<py::gil_scoped_release>())
            .def_property_readonly("available_vehicles", &neptus::GCS::available_vehicles);

    py::class_<neptus::PlanExecutionReport>(m, "PlanExecutionReport")
            .def("__repr__", [](neptus::PlanExecutionReport& self) -> std::string {
                std::stringstream ss;
                ss << "PlanExecutionReport(" << self.timestamp << ", "
                   << self.plan_id << ", "
                   << static_cast<uint8_t>(self.state) << ")";
                return ss.str();
            })
            .def_readonly("timestamp", &neptus::PlanExecutionReport::timestamp)
            .def_readonly("plan_id", &neptus::PlanExecutionReport::plan_id)
//            .def_readonly("state", &neptus::PlanExecutionReport::state)
            .def_readonly("vehicles", &neptus::PlanExecutionReport::vehicles);

    py::class_<neptus::UAVStateReport>(m, "UAVStateReport")
            .def("__repr__", [](neptus::UAVStateReport& self) -> std::string {
                std::stringstream ss;
                ss << "UAVStateReport(t="
                   << self.timestamp << ", u="
                   << self.uav_id << ", ("
                   << self.lat << ", "
                   << self.lon << ", "
                   << self.height << "), ("
                   << self.phi << ", "
                   << self.theta << ", "
                   << self.psi << "), ("
                   << self.vx << ", "
                   << self.vy << ", "
                   << self.vz << "))";
                return ss.str();
            })
            .def_readonly("uav_id", &neptus::UAVStateReport::uav_id)
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
