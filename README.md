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

    ./docker/run.sh make_container

To start a shell in the container with the the code repository and the data repository mounted:

    ./docker/run.sh [run]
    
Other options to the `run.sh` script include:

- `build`: compile the project
- `autobuild`: Recompile the project each time a source file is changed
- `test`: Run unit tests