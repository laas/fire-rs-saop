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

#ifndef PLANNING_CPP_CPP_PY_UTILS_HPP
#define PLANNING_CPP_CPP_PY_UTILS_HPP

#include <pybind11/pybind11.h>

#include <pybind11/stl.h> // for conversions between c++ and python collections
#include <pybind11/numpy.h> // support for numpy arrays

namespace py = pybind11;

/** Converts a numpy array to a vector */
template<class T>
std::vector<T> as_vector(py::array_t<T, py::array::c_style | py::array::forcecast> array) {
    std::vector<T> data(array.size());
    for (ssize_t x = 0; x < array.shape(0); x++) {
        for (ssize_t y = 0; y < array.shape(1); y++) {
            data[x + y * array.shape(0)] = *(array.data(x, y));
        }
    }
    return data;
}

/** Converts a vector to a 2D numpy array. */
template<class T>
py::array_t<T> as_nparray(std::vector<T> vec, size_t x_width, size_t y_height) {
    ASSERT(vec.size() == x_width * y_height);
    py::array_t<T, py::array::c_style | py::array::forcecast> array(std::vector<size_t> { x_width, y_height });
    auto s_x_width = static_cast<ssize_t>(x_width);
    auto s_y_height = static_cast<ssize_t>(y_height);
    for (ssize_t x = 0; x < s_x_width; x++) {
        for (ssize_t y = 0; y < s_y_height; y++) {
            *(array.mutable_data(x, y)) = vec[x + y * x_width];
        }
    }
    return array;
}

#endif //PLANNING_CPP_CPP_PY_UTILS_HPP
