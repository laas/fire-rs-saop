FireRS Situation Assessment and Observation Planning
====================================================

Build
-----

SAOP uses cython for the implementation of the fire propagation models (the `firemodel` module).
Before using the project, you should therefore build the cython module using the setup script:

    python setup.py build_ext --inplace

This will generate shared library (.so) in the project folder and you should then be able to use the `firemodel` module as if it was plain python.


Docker
------

A Dockerfile is provided to set up an environment with windninja and all python dependencies.

To build the container:

    ./docker/run.sh build

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.sh [start]
    
Makefile
--------

All most commons operations are available in the Makefile. Targets include

- `build`: compile the project
- `autobuild`: Recompile the project each time a source file is changed (requires "when-changed")
- `test`: Run unit tests
- `test-cpp`: Run unittests for the C++ module only
- `docker`: starts a shell in the docker container.
- `docker_build_container`: Build/rebuild the docker image. 

A typical workflow would be to start 