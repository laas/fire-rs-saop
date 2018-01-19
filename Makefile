
build: build-release

# Builds all python extensions 
build-release: FORCE
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Release -DWITH_IMC_INTERFACE=ON ..
	cd build && make

build-debug: FORCE
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Debug -DWITH_IMC_INTERFACE=ON ..
	cd build && make
	
build-testing: FORCE
	mkdir -p build
	cd build && cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON ..
	cd build && make

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
	./build/cpp/tests

benchmark: build-release FORCE
	cd python && PYTHONPATH="${PYTHONPATH}:./python/" python3 fire_rs/planning/benchmark.py

# Remove the build folder and clean python source dir
clean:
	rm -r build || true
	cd python && python3 setup.py clean --all || true
	rm python/fire_rs/uav_planning.cpython-*.so || true
	rm python/fire_rs/neptus_interface.cpython-*.so || true
	rm python/fire_rs/firemapping.cpython-*.so || true

# phantom task that always need to be run
FORCE: ;
