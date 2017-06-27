#!/bin/bash

# A simple script to build and start a docker container for Fire-RS SAOP
# The script expects one parameter 'build' or 'start', defaulting to 'start'
# When invoked with 'build', the container will be built
# When invoked with 'start', the container will be started to give a bash prompt in the container

DOCKER_DIR="$(realpath $(dirname ${BASH_SOURCE[0]}))"
ROOT_DIR="$(realpath $(dirname ${DOCKER_DIR}))"

CONTAINER_NAME='saop'

CONTAINER_START_CMD="docker run -it --cap-add=SYS_PTRACE -v ${ROOT_DIR}:/home/saop/code -v ${FIRERS_DATA}:/home/saop/data -e DISPLAY=${DISPLAY} -v /tmp/.X11-unix:/tmp/.X11-unix ${CONTAINER_NAME}"

case $1 in
    'build')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo ""
        docker build -t ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'rebuild')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo ""
        docker build --no-cache=true -t ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'start'|'')
        echo "Running container ${CONTAINER_NAME} using:"
        echo "  ${ROOT_DIR} as source directory (mapped to /home/saop/code)"
        echo "  ${FIRERS_DATA} as data directory (mapped to /home/saop/data)"
        echo ""
        eval ${CONTAINER_START_CMD}
        ;;
    *)
        echo "Error: Expected 'make_container', 'start' or 'build' as parameter. Got: $1"
        ;;
esac


