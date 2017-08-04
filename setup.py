#!/usr/bin/env python3
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
    url="http://fire-rs.com",
    packages=find_packages(),
    license="GPLv3+",
    install_requires=['affine', 'GDAL', 'numpy', 'pandas', 'pytz', 'matplotlib', 'sklearn', 'pybind11'],
    ext_modules=[
        Extension("fire_rs.firemodel.rothermel", sources=["fire_rs/firemodel/rothermel.pyx"]),
        Extension("fire_rs.firemodel.environment", sources=["fire_rs/firemodel/environment.pyx"]),
        Extension("fire_rs.firemodel.fireshapes", sources=["fire_rs/firemodel/fireshapes.pyx"]),
        # Extension("fire_rs.firemodel.propagation", sources=["fire_rs/firemodel/propagation.pyx"]),
        Extension(
            name='fire_rs.uav_planning',
            sources=['fire_rs/planning-cpp/src/ext/dubins.cpp', 'fire_rs/planning-cpp/src/python_interface.cpp'],
            depends=glob.glob(os.path.join('fire_rs', 'planning-cpp', 'src', '**', '*.h'), recursive=True),
            include_dirs=[pybind11.get_include(False), pybind11.get_include(True)],  # Path to pybind11 headers
            extra_compile_args=["-std=c++11", "-O0", "-g" , "-Wall", "-Wno-deprecated"],
            language='c++',
            undef_macros=['NDEBUG'],  # enable assertions in compiled C++ code
        ),
    ],
    cmdclass={'build_ext': build_ext}
)
