#!/bin/bash

# start a docker container in the background
sh docker_start.sh $1

pwd
sh docker_exec.sh "make build-release"

sh docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --format svg --vns-conf=scripts/benchmarking/vns_confs.json --vns=base --elevation=flat"
sh docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --format svg --vns-conf=scripts/benchmarking/vns_confs.json --vns=full --elevation=flat"

docker rm -f saop
