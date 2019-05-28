# FireRS Situation Assessment and Observation Planning

*FireRS Situation Assessment and Observation Planning (SAOP)* is composed of several python packages:

 - ```fire_rs.geodata```: geographic and environmental data manipulation
 - ```fire_rs.firemodel```: wildfire propagation
 - ```fire_rs.planning```: high-level interface for UAV planning
 - ```fire_rs.monitoring```: high-level functionalities of the wildfire monitoring mission 
 - ```fire_rs.simulation```: create and control a synthetic wildfire in the Morse robotic simulator
 
   *Those can be found in the ```python``` directory.*

A C++ library (___libsaop___) implementing:
 - A VNS-based observation planning algorithm for wildfire monitoring.
 - The communication with [Neptus](https://github.com/LSTS/neptus) and [Dune](https://github.com/LSTS/dune) software for the operation of unmmaned vehicles.
 - A trivial wildfire mapping algorithm.
 
   The functionality provided by _libsaop_ is also exposed trough three python interfaces:
   - *uav_planning*: Core planning algorithm
   - *neptus*: Communication with Neptus and Dune
   - *firemapping*: mapping algorithms
 
And a ROS pacakge (___supersaop___) for real time execution of SAOP with real or simulated UAVs in a real or synthetic wildfire scenario.

## Using SAOP
[![Build Status](https://travis-ci.org/laas/fire-rs-saop.svg?branch=master)](https://travis-ci.org/laas/fire-rs-saop)
### Requirements

The core functions of FireRS SAOP require running a GNU/Linux operating system (Ubuntu 18.04 or Fedora 29 recommended) with:

 - cmake
 - a C++11 compiler (or later)
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
    * scikit-image
    * scipy

Optionally, for the real time execution:

 - [ROS](http://www.ros.org/) (if installing from source, only the *ROS-Comm* variant is needed)
 - [Neptus](https://github.com/lsts/neptus) and [Dune](https://github.com/lsts/dune) 

*pybind11 is present as a submodule of the current repository. To retrieve it:*

    git submodule init
    git submodule update

### Local build

Just do `make build`.

In detail, targets for the Makefile are:

- `build`: compile the project (default target)
- `build-debug`: compile *libsaop* with debugging symbols
- `build-testing`: compile including *libsaop* tests with debugging symbols in the build
- `autobuild`: recompile the project each time a source file is changed (requires "when-changed")
- `benchmark`: Create a random fire scenario and benchmark observation plan search.
- `doc`: generate *libsaop* python interface html documentation (needs *sphinx*)
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

The file `docker/Dockerfile` contains all the steps require to retrieve most of the dependencies on an Ubuntu 16.04 distribution.

A Dockerfile is provided to set up an environment with windninja and all python dependencies.

To build the container:

    ./docker/run.sh build

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.sh [start]
