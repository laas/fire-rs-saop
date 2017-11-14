#!/bin/bash

for vns in {"base","full"}; do
    python3 ../python/fire_rs/planning/benchmark.py --vns=$vns --name=default --elevation=flat
done
