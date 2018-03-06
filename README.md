FireRS Situation Assessment and Observation Planning
====================================================

Build process details.
-----

[![Build Status](https://travis-ci.org/fire-rs-laas/fire-rs-saop.svg?branch=master)](https://travis-ci.org/fire-rs-laas/fire-rs-saop)

SAOP require pybind11 which is present as a submodule of the current repository. To retrieve it:

    git submodule init
    git submodule update

SAOP has two software parts. A C++ backend in `cpp/` used for generating plans and a python module in `python/` that handles geographic data, simulate wildfire and provide an high-level API.
The build system uses CMake and be used with:

    mkdir build/
    cd build
    cmake ..
    make
    
This process will:
 - build a C++ backend with python bindings
 - build the Cython modules in `python/`
 - ensure that the produced artifacts are located in the `python/` source directory so that the python scripts can be invoke in straightforward manner. 

SAOP has a number of dependencies that you obtain by using the docker image (below).
Otherwise, the file `docker/Dockerfile` contains all the steps require to retrieve all dependencies on an Ubuntu 16.04 distribution.

Optionally, C++ unitary tests can be built if the argument -DBUILD_TESTING=ON is passed to CMake

Docker
------

A Dockerfile is provided to set up an environment with windninja and all python dependencies.

To build the container:

    ./docker/run.sh build

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.sh [start]
    

Makefile
--------

Most commons operations are available in the Makefile. Targets include

- `build`: compile the project (default target) 
- `autobuild`: Recompile the project each time a source file is changed (requires "when-changed")
- `test-python`: Run all python unit tests
- `test-cpp`: Run unittests specific to the C++ module only
- `test-python-cpp`: Run test interfacing the python and C++ codes.
- `docker`: starts a shell in the docker container.
- `docker_build_container`: Build/rebuild the docker image. 


A typical workflow would be to launch the docker container(`make docker`) and the build the project inside it.A typical workflow would be to launch the docker container(`make docker`) and the build the project inside it.
