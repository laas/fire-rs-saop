#!/bin/bash


DATA_REPO=${FIRERS_DATA:-ssh://git@redmine.laas.fr/laas/users/simon/fire-rs/fire-rs-data.git}
CODE_REPO="`pwd`/../.."

# create a temporary directory in which to run the benchmark
DIR_TEMPLATE="saop_bench_`date +%F-%H-%M-%S`_XXXX"
DIR=`mktemp -d -t ${DIR_TEMPLATE} --tmpdir=.`

echo "Workdir: ${DIR}"
cd $DIR

# fetch clean data and code repository (non commited changes are ignored!)
echo "Fetching repositories"
if test -d $DATA_REPO; then
	echo "Making a symbolic link to $DATA_REPO"
    ln --symbolic $DATA_REPO data
else
	echo "FIRERS_DATA is not defined, trying to clone from "$DATA_REPO
	if ! git clone $DATA_REPO data; then
		echo "'git clone' failed! exiting..."
		exit 1
	fi
fi

git clone $CODE_REPO fire-rs-saop
cd fire-rs-saop
git submodule update --init --recursive
cd ..

# start container in the background
echo "Starting docker container 'saop'"
docker run -d -it --user=${USER_ID}:${GROUP_ID} --name saop \
       -v `pwd`/fire-rs-saop:/home/saop/code:z \
       -v `pwd`/data:/home/saop/data:z \
       abitmonn/firers-saop-dev

# display running containers
docker ps
