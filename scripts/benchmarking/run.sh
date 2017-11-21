#!/bin/bash

DATA_REPO=/tmp/saop_data #:-ssh://git@redmine.laas.fr/laas/users/simon/fire-rs/fire-rs-data.git}
CODE_REPO="`pwd`/../.."
EXEC="$(pwd)/docker_exec.sh"

# create a temporary directory in which to run the benchmark
DIR_TEMPLATE="saop_bench_`date +%F-%H-%M-%S`_XXXX"
DIR=`mktemp -d -t ${DIR_TEMPLATE} --tmpdir=.`

echo "Workdir: ${DIR}"
cd $DIR

# fetch clean data and code repository (non commited changes are ignored!)
echo "Fetching repositories"
if test -d $DATA_REPO; then
	echo "~/data. Will be mounted to $DATA_REPO"
	ln -s $DATA_REPO data
elif test -d ${FIRERS_DATA}; then
    echo "Copying ${FIRERS_DATA} to ${DATA_REPO}"
    cp -r ${FIRERS_DATA} ${DATA_REPO}
else
	echo "DATA_REPO does not exists. Aborting"
	exit 1
fi

echo "Copying code repository"
rsync -a $CODE_REPO fire-rs-saop --exclude scripts/benchmarking/saop_bench*
#git clone $CODE_REPO fire-rs-saop
echo "Cleaning code repository"
cd fire-rs-saop && make clean
#git submodule update --init --recursive
cd ..



# start container in the background
echo "Starting docker container 'saop'"
docker run -d -it --user=${USER_ID}:${GROUP_ID} --name saop \
       -v `pwd`/fire-rs-saop:/home/saop/code:z \
       -v ${DATA_REPO}:/home/saop/data:z \
       abitmonn/firers-saop-dev

# display running containers
docker ps

echo "Docker container 'saop' started"
sleep 2


mkdir -p data/benchmark_archive && mv data/benchmark/* data/benchmark_archive/

bash ${EXEC} "make build-release"

bash ${EXEC} "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=base --elevation=flat"
bash ${EXEC} "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=full --elevation=flat"
#bash docker_exec.sh "python3 python/fire_rs/planning/benchmark.py --vns-conf=scripts/benchmarking/vns_confs.json --vns=with_smoothing --elevation=flat"

cp -r data/benchmark results

docker rm -f saop
