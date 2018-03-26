BUILD_FOLDER=build
BUILD_DEBUG_FOLDER=build-debug
BUILD_TESTING_FOLDER=build-testing

build: build-release

# Builds all python extensions 
build-release: FORCE
	mkdir -p $(BUILD_FOLDER)
	cd $(BUILD_FOLDER) && cmake -DCMAKE_BUILD_TYPE=Release  ..
	cd $(BUILD_FOLDER) && make

build-debug: FORCE
	mkdir -p $(BUILD_DEBUG_FOLDER)
	cd $(BUILD_DEBUG_FOLDER) && cmake -DCMAKE_BUILD_TYPE=Debug ..
	cd $(BUILD_DEBUG_FOLDER) && make
	
build-testing: FORCE
	mkdir -p $(BUILD_TESTING_FOLDER)
	cd $(BUILD_TESTING_FOLDER) && cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DWITH_IMC_INTERFACE=ON ..
	cd $(BUILD_TESTING_FOLDER) && make

# rebuilds project each time a C++ source is modified
# this requires the "when-changed" program" that is installed in the docker container
# look in docker/Dockerfile to see how to install it.
autobuild: FORCE
	when-changed cpp/src/* -c "make && echo 'BUILT\n\nWaiting for changes...'"

# launches the docker container
docker: FORCE
	docker/run.sh start

# Rebuilds the docker container (needed only on modification of 
docker_build_container:
	docker/run.sh build

# Run all unittests
test-python: build-testing FORCE
	cd python && python3 -m unittest

# Run python test for "planning-cpp"
test-python-cpp: build-testing FORCE
	cd python && python3 -m unittest python.fire_rs.planning.test_uav_planning_cpp

test-cpp: build-testing
	./$(BUILD_TESTING_FOLDER)/cpp/tests

benchmark: build-release FORCE
	cd python && PYTHONPATH="${PYTHONPATH}:./python/" python3 fire_rs/planning/benchmark.py

doc: build FORCE
	cd doc && make html

# Remove the build folder and clean python source dir
clean:
	rm -r $(BUILD_FOLDER) || true
	rm -r $(BUILD_DEBUG_FOLDER) || true
	rm -r $(BUILD_TESTING_FOLDER) || true
	cd python && python3 setup.py clean --all || true
	rm python/fire_rs/uav_planning.cpython-*.so || true
	rm python/fire_rs/neptus_interface.cpython-*.so || true
	rm python/fire_rs/firemapping.cpython-*.so || true

# phantom task that always need to be run
FORCE: ;
