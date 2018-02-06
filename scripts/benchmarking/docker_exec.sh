#!/bin/bash

# simple script to run a command in the "saop_$1" container

PARAMS="${@:2}"

docker exec -it --user=${USER_ID}:${GROUP_ID} saop_$1 /bin/bash -c "$PARAMS"
