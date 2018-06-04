# FireRS Situation Assessment and Observation Planning

FireRS Situation Assessment and Observation Planning (SAOP) is composed of several python modules:

 - *geodata*: geographic and environmental data manipulation
 - *firemodel*: wildfire propagation
 - *planning*: high-level interface for UAV planning

and a C++ library (_libsaop_) with python interfaces:

 - *uav_planning*: UAV planning core algorithms
 - *neptus*: Communication with [Neptus command and control infrastructure](https://github.com/lsts/neptus)
 - *firemapping*: Wildfire mapping algorithms

## Running SAOP
[![Build Status](https://travis-ci.org/laas/fire-rs-saop.svg?branch=master)](https://travis-ci.org/laas/fire-rs-saop)
### Requirements

A GNU/Linux operating system with:

 - cmake
 - a C++11 compiler (at least)
 - Boost
 - python 3.5
 - Cython
 - [pybind11](https://github.com/pybind/pybind11/)
 - gdal
 - [windninja](https://github.com/firelab/windninja)
 - The following python packages:
    * affine
    * GDAL
    * joblib
    * matplotlib
    * numpy
    * pybind11
    * pytz
    * ...

Optionally:

 - [Neptus](https://github.com/lsts/neptus) and [Dune](https://github.com/lsts/dune) for UAV simulation and plan execution

The file `docker/Dockerfile` contains all the steps require to retrieve all dependencies on an Ubuntu 16.04 distribution.

pybind11 is present as a submodule of the current repository. To retrieve it:

    git submodule init
    git submodule update

### On your system

Just do `make build`.

In detail, targets for the Makefile are:

- `build`: compile the project (default target)
- `build-debug`: compile *libsaop* with debugging symbols
- `build-testing`: compile including *libsaop* tests with debugging symbols in the build
- `autobuild`: recompile the project each time a source file is changed (requires "when-changed")
- `benchmark`: Create a random fire scenario and benchmark observation plan search.
- `doc`: generate *libsaop* python interface html documentation
- `clean`: remove the build folder and python artifacts
- `test-python`: run all python unit tests
- `test-cpp`: run unittests specific to the C++ module only
- `test-python-cpp`: run test interfacing the python and C++ codes.
- `docker`: starts a shell in the docker container.
- `docker_build_container`: Build/rebuild the docker image. 

Additionally, SAOP cmake build can be configured manually with:

    mkdir build/
    cd build
    cmake ..
    make

This process will:
 - build a C++ backend with python bindings
 - build the Cython modules in `python/`
 - ensure that the produced artifacts are located in the `python/` source directory so that the python scripts can be invoke in straightforward manner.

### On Docker


A Dockerfile is provided to set up an environment with windninja and all python dependencies.

To build the container:

    ./docker/run.sh build

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.sh [start]
