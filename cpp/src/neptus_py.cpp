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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

#include "neptus/saop_neptus.hpp"


namespace py = pybind11;


PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

PYBIND11_MODULE(neptus_interface, m) {
    m.doc() = "Python module for interfacing with neptus/dune software";

    m.def("upload_plan", SAOP::neptus::send_plan_to_dune, py::arg("ip"), py::arg("port"),
          py::arg("plan"), py::arg("plan_id"), py::arg("segment_extension"), py::arg("sampled"));

    m.def("start_plan", SAOP::neptus::send_plan_start_request, py::arg("ip"), py::arg("port"),
          py::arg("plan_id"));

    py::class_<SAOP::neptus::DuneLink>(m, "DuneLink")
            .def(py::init<std::string, std::string>(), py::arg("ip"), py::arg("port"));

}

#endif //PLANNING_CPP_PYTHON_NEPTUS_H
