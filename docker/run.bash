#!/bin/bash

# A simple script to build and start a docker container for Fire-RS SAOP
# The script expects one parameter 'build' or 'start', defaulting to 'start'
# When invoked with 'build', the container will be built
# When invoked with 'start', the container will be started to give a bash prompt in the container

DOCKER_DIR="$(realpath $(dirname ${BASH_SOURCE[0]}))"
# FireRS SAOP root dir is this script parent. It replaces the code already present in the container
ROOT_DIR="$(realpath $(dirname ${DOCKER_DIR}))"

# X11 authorization stuff
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

CONTAINER_NAME='fire-rs-saop'

CONTAINER_START_CMD="docker run -it --user=saop:saop --cap-add=SYS_PTRACE \
--volume=${ROOT_DIR}:/home/saop/code:z \
--volume=${FIRERS_DATA}:/home/saop/data:z \
--env="DISPLAY=${DISPLAY}" --env="XAUTHORITY=${XAUTH}" \
--volume=$XSOCK:$XSOCK:rw --volume=$XAUTH:$XAUTH:rw \
${CONTAINER_NAME}"

USER_UID="$(id -u)"
USER_GROUP="$(id -g)"

BUILD_OPTIONS="--build-arg USER_UID=${USER_UID} --build-arg USER_GROUP=${USER_GROUP} -t"

case $1 in
    'build')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo "Cloning github laas:fire-rs-saop master as source directory (/home/saop/code)"
        echo ""
        docker build ${BUILD_OPTIONS} ${CONTAINER_NAME} ${DOCKER_DIR}
        ;;
    'rebuild')
        echo "Building container $CONTAINER_NAME from directory $DOCKER_DIR"
        echo "Cloning github laas:fire-rs-saop master as source directory (/home/saop/code)"
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
        echo "Error: Expected 'build', 'rebuild' or 'start' as parameter. Got: $1"
        ;;
esac
