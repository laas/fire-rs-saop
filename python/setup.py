#!/usr/bin/env python3

# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import glob
import os

from setuptools import find_packages, setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

import pybind11

setup(
    name="Fire RS",
    version="0.1",
    description="FireRS situation assessment and observation planning (SAOP) software",
    url="https://github.com/laas/fire-rs-saop",
    packages=find_packages(),
    license="BSD 2-clause",
    install_requires=['affine',
                      'GDAL',
                      'joblib',
                      'matplotlib',
                      'numpy',
                      'pandas',
                      'pybind11',
                      'pymorse',
                      'pytz',
                      'scikit-image',
                      'scipy'],
    ext_modules=[
        Extension("fire_rs.firemodel.rothermel", sources=["fire_rs/firemodel/rothermel.pyx"]),
        Extension("fire_rs.firemodel.environment", sources=["fire_rs/firemodel/environment.pyx"]),
        Extension("fire_rs.firemodel.fireshapes", sources=["fire_rs/firemodel/fireshapes.pyx"]),
        # Extension("fire_rs.firemodel.propagation", sources=["fire_rs/firemodel/propagation.pyx"]),
        # Extension(
        #     name='fire_rs.uav_planning',
        #     sources=['fire_rs/planning-cpp/src/ext/dubins.cpp', 'fire_rs/planning-cpp/src/python_interface.cpp'],
        #     depends=glob.glob(os.path.join('fire_rs', 'planning-cpp', 'src', '**', '*.h'), recursive=True),
        #     include_dirs=[pybind11.get_include(False), pybind11.get_include(True)],  # Path to pybind11 headers
        #     extra_compile_args=["-std=c++11", "-O0", "-g" , "-Wall", "-Wno-deprecated"],
        #     language='c++',
        #     undef_macros=['NDEBUG'],  # enable assertions in compiled C++ code
        # ),
    ],
    cmdclass={'build_ext': build_ext}
)
