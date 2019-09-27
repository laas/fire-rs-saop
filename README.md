# FireRS Situation Assessment and Observation Planning

*FireRS Situation Assessment and Observation Planning (SAOP)* is a software for planning, execution and supervision of wildfire monitoring missions with fleets of UAVs.

FireRS SAOP is composed of several python packages:

 - ```fire_rs.geodata```: geographic and environmental data manipulation
 - ```fire_rs.firemodel```: wildfire propagation
 - ```fire_rs.planning```: high-level interface for UAV planning
 - ```fire_rs.monitoring```: high-level functionalities of the wildfire monitoring mission 
 - ```fire_rs.simulation```: create and control a synthetic wildfire in the Morse robotic simulator
 
   *Those can be found in the ```python``` directory.*

A C++ library (___libsaop___) implementing:
 - A VNS-based observation planning algorithm for wildfire monitoring.
 - The communication with [Neptus](https://github.com/LSTS/neptus) and [Dune](https://github.com/LSTS/dune) software for the operation of unmanned vehicles.
 - A trivial wildfire mapping algorithm.
 
   The functionality provided by _libsaop_ is also exposed trough three python interfaces:
   - *uav_planning*: Core planning algorithm
   - *neptus*: Communication with Neptus and Dune
   - *firemapping*: mapping algorithms
 
And a ROS pacakge (___supersaop___) for real time execution of SAOP with real or simulated UAVs in a real or synthetic wildfire scenario.

## SAOP environment setup

### 1. On Docker
The easiest way to start with SAOP is using docker.

To build the container:

    ./docker/run.bash build

The file `docker/Dockerfile` contains all the steps to build a SAOP-ready Ubuntu 18.04 image.

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.bash start

The `/docker/run.bash` is the recommended way to build and run the SAOP docker container as it sets up some X11 bindings from the host.

### 2. Local software requirements

_(If you are using docker, skip this section and continue with **Geographic data**)_

The core functions of FireRS SAOP require running a GNU/Linux operating system (Ubuntu 18.04 and Fedora 29 tested) with:

 - cmake
 - a C++11 compiler
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
    * pandas
    * pybind11
    * pytz
    * scikit-image
    * scipy

Using Ubuntu, run as *root*:

    apt install g++ cmake python3 cython3 python3-gdal python3-setuptools\
    python3-pip python3-affine python3-tz python3-pandas python3-numpy\
    python3-matplotlib libboost-all-dev libgdal-dev
    
    pip3 install pybind11 joblib scikit-image scipy

*pybind11 is present as a submodule of the current repository. To retrieve it:*

    git submodule init
    git submodule update
    
#### 3.1 Real time execution  requirements

Optionally, for the real time execution:

 - [ROS](http://www.ros.org/)
 - [Neptus](https://github.com/lsts/neptus) and [Dune](https://github.com/lsts/dune) 
    
### 3. Geographic data

On top of the software requierements, some geographic data are needed in order to propagate a fire in the area of your choice.
You should set an environment variable ```FIRERS_DATA``` containing the path to a folder with 3 folders inside following this structure:

 - ```firers_data/```
   - ```dem/```
   - ```landcover/```
   - ```wind/```

Some sample files can be found in: [https://github.com/laas/fire-rs-data](https://github.com/laas/fire-rs-data).
For using SAOP in other regions you have to acquire the corresponding DEM and lancover maps for the area.
*(A good source for Europe are the European Digital Elevation Model (EU-DEM) and Corine Land Cover (CLC) 2006)*

Additionally, an environment variable ```WINDNINJA_CLI_PATH``` should be set to the path where the windninja executable can be found.
windninja_s 
Windninja will be creating files in the ```wind``` folder as needed with the local wind for the DEMs in the ```dem``` folder. 

## Building saop
[![Build Status](https://travis-ci.org/laas/fire-rs-saop.svg?branch=master)](https://travis-ci.org/laas/fire-rs-saop)

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

### License and citations

    Copyright (c) 2017-2019, CNRS-LAAS
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
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


If you use fire-rs-saop in a scientific publication, we would appreciate citations to:

 - Rafael Bailon-Ruiz, Arthur Bit-Monnot, Simon Lacroix. [**Planning to Monitor Wildfires with a Fleet of UAVs**](https://hal.laas.fr/hal-01852176), *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, Oct 2018, Madrid, Spain. DOI: [10.1109/IROS.2018.8593859](https://doi.org/10.1109/IROS.2018.8593859)
