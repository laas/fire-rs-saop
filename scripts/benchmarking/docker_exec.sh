#!/bin/bash

# simple script to run a command in the "saop" container

PARAMS="$@"

docker exec -it --user=${USER_ID}:${GROUP_ID} saop /bin/bash -c "$PARAMS"
