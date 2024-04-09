.PHONY: default prepare build launch clean

default:
	@echo 'make prepare'
	@echo '    Install required dependencies for this project.'
	@echo
	@echo 'make build'
	@echo '    Build this project.'
	@echo
	@echo 'make clean'
	@echo '    Clean up built binaries.'

prepare:
	git submodule update --init --recursive

	cargo install --git https://github.com/jerry73204/cargo-ament-build.git
	pip3 install -U git+https://github.com/jerry73204/colcon-ros-cargo.git@merge-colcon-cargo

	rosdep update --rosdistro=humble && \
	rosdep install -y --from-paths src --ignore-src -r

build:
	colcon build \
		--merge-install \
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release \
		--cargo-args --release

launch:
	ros2 launch launch/f1eigth.launch.yaml

clean:
	rm -rf build install log
