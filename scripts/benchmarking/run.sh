#!/bin/bash

# start a docker container in the background
sh docker_start.sh $1

pwd
bash docker_exec.sh $1 "make build-release"

bash docker_exec.sh $1 "python3 python/fire_rs/planning/benchmark.py --format svg --vns-conf=scripts/benchmarking/vns_confs.json --vns=base --elevation=flat"
bash docker_exec.sh $1 "python3 python/fire_rs/planning/benchmark.py --format svg --vns-conf=scripts/benchmarking/vns_confs.json --vns=full --elevation=flat"

docker rm -f saop_$1
