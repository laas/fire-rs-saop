
# Builds all python extensions 
build: FORCE
	python3 setup.py build_ext --inplace

# rebuilds project each time a C++ source is modified
# this requires the "when-changed" program" that is installed in the docker container
# look in docker/Dockerfile to see how to install it.
autobuild: FORCE
	when-changed fire_rs/planning-cpp/src/* -c "python3 setup.py build_ext --inplace && echo 'BUILT\n\nWaiting for changes...'"

# launches the docker container
docker: FORCE
	docker/run.sh start

# Rebuilds the docker container (needed only on modification of 
docker_build_container:
	docker/run.sh build

# Run all unittests
test: build FORCE
	python3 -m unittest

# Run test for "planning-cpp"
test-cpp: build FORCE
	python3 -m unittest fire_rs.planning.test_uav_planning_cpp

benchmark: build FORCE
	PYTHONPATH="${PYTHONPATH}:." python3 fire_rs/planning/benchmark.py



# phantom task that always need to be run
FORCE: ;
