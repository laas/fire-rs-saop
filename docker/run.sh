#!/bin/bash

# A simple script to build and run a docker container for Fire-RS SAOP
# The script expects one parameter 'build' or 'run', defaulting to 'run' if none is given
# When invoked with 'build', the container will be built
# When invoked with 'run', the container will be started to give a bash prompt in the container

DOCKER_DIR="$(realpath $(dirname ${BASH_SOURCE[0]}))"
ROOT_DIR="$(realpath $(dirname ${DOCKER_DIR}))"

CONTAINER_NAME='saop'

CONTAINER_START_CMD="docker run -it --cap-add=SYS_PTRACE -v ${ROOT_DIR}:/home/saop/code -v ${FIRERS_DATA}:/home/saop/data -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix ${CONTAINER_NAME}"

BUILD_CMD='python3 setup.py build_ext --inplace'
AUTO_BUILD_CMD="when-changed fire_rs/planning-cpp/src/* -c ${BUILD_CMD}"
TEST_CMD="python3 -m unittest fire_rs.planning.test_uav_planning_cpp"

case $1 in
    'make_container')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo ""
        docker build -t ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'build')
        echo 'Building the project once (use autobuild for repeated)'
        cmd="${CONTAINER_START_CMD} ${BUILD_CMD}"
        echo "Running: $cmd"
        eval $cmd
        ;;
    'autobuild')
        echo 'Building the project one each source changes.'
        cmd="${CONTAINER_START_CMD} ${AUTO_BUILD_CMD}"
        echo "Running: $cmd"
        eval $cmd
        ;;
    'test')
        cmd="${CONTAINER_START_CMD} ${TEST_CMD}"
        echo "Running: $cmd"
        eval $cmd
        ;;
    'run'|'')
        echo "Running container ${CONTAINER_NAME} using:"
        echo "  ${ROOT_DIR} as source directory (mapped to /home/saop/code)"
        echo "  ${FIRERS_DATA} as data directory (mapped to /home/saop/data)"
        echo ""
        eval ${CONTAINER_START_CMD}
        ;;
    *)
        echo "Error: Expected 'make_container', 'run', 'build', 'autobuild', 'test' or '' as first parameter. Got: $1"
        ;;
esac


