#!/bin/bash

# A simple script to build and start a docker container for Fire-RS SAOP
# The script expects one parameter 'build' or 'start', defaulting to 'start'
# When invoked with 'build', the container will be built
# When invoked with 'start', the container will be started to give a bash prompt in the container

DOCKER_DIR="$(realpath $(dirname ${BASH_SOURCE[0]}))"
ROOT_DIR="$(realpath $(dirname ${DOCKER_DIR}))"

XSOCK=/tmp/.X11-unix

CONTAINER_NAME='saop'

CONTAINER_START_CMD="docker run -it --user=saop:saop --cap-add=SYS_PTRACE -v ${ROOT_DIR}:/home/saop/code:z -v ${FIRERS_DATA}:/home/saop/data:z -e DISPLAY=${DISPLAY} -v ${XSOCK} ${CONTAINER_NAME}"

USER_UID="$(id -u)"
USER_GROUP="$(id -g)"

BUILD_OPTIONS="--build-arg USER_UID=${USER_UID} --build-arg USER_GROUP=${USER_GROUP} -t"

case $1 in
    'build')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo ""
        docker build ${BUILD_OPTIONS} ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'rebuild')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo ""
        docker build --no-cache=true ${BUILD_OPTIONS} ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'start'|'')
        echo "Running container ${CONTAINER_NAME} using:"
        echo "  ${ROOT_DIR} as source directory (mapped to /home/saop/code)"
        echo "  ${FIRERS_DATA} as data directory (mapped to /home/saop/data)"
        echo ""
        echo ${CONTAINER_START_CMD}
        eval ${CONTAINER_START_CMD}
        ;;
    *)
        echo "Error: Expected 'make_container', 'start' or 'build' as parameter. Got: $1"
        ;;
esac
