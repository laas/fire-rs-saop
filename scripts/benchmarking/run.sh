#!/bin/bash

# start a docker container in the background
sh docker_start.sh

pwd
sh docker_exec.sh "make build-release"

sh docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=base --elevation=flat"
sh docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=full --elevation=flat"
sh docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=with_smoothing --elevation=flat"

docker rm -f saop
