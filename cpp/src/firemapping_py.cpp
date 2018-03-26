/* Copyright (c) 2018, CNRS-LAAS
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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

#include "firemapping/ghostmapper.hpp"
#include "cpp_py_utils.hpp"

namespace py = pybind11;


PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)

PYBIND11_MODULE(firemapping, m) {
    m.doc() = "Fire mapping algorithms";

    py::class_<GhostFireMapper<double>>(m, "GhostFireMapper")
            .def(py::init<std::shared_ptr<FireData>>(), py::arg("environment_gt"))

            .def_property_readonly("firemap", &GhostFireMapper<double>::firemap)
            .def_property_readonly("observed", &GhostFireMapper<double>::observed)

            .def("observe", (void (GhostFireMapper<double>::*)(const Trajectory&)) &GhostFireMapper<double>::observe,
                 py::arg("trajectory"))
            .def("observe", (void (GhostFireMapper<double>::*)(const Trajectories&)) &GhostFireMapper<double>::observe,
                 py::arg("trajectories"))
            .def("observe", (void (GhostFireMapper<double>::*)(const vector<Waypoint3d>&, const vector<double>&,
                                                               const UAV&)) &GhostFireMapper<double>::observe,
                 py::arg("waypoint3d_list"), py::arg("time_list"), py::arg("uav"))

            .def("observed_fire",
                 (GenRaster<double> (GhostFireMapper<double>::*)(const std::vector<Waypoint3d>&,
                                                                 const std::vector<double>&,
                                                                 const UAV&) const)
                         &GhostFireMapper<double>::observed_fire, py::arg("waypoint3d_list"), py::arg("time_list"),
                 py::arg("uav"));

}

#endif //PLANNING_CPP_PYTHON_NEPTUS_H