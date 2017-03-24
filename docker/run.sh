#!/bin/bash

docker run -it -v $PWD:/home/saop/code -v ~/.ssh:/home/saop/.ssh -v $FIRERS_DATA:/home/saop/data saop 
