.PHONY: default prepare build launch clean
SHELL := /bin/bash

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

	source /opt/ros/humble/setup.sh && \
	rosdep update --rosdistro=humble && \
	rosdep install -y --from-paths src --ignore-src -r

build:
	source /opt/ros/humble/setup.sh && \
	colcon build \
		--merge-install \
		--symlink-install \
		--cmake-args -DCMAKE_BUILD_TYPE=Release \
		--cargo-args --release

launch:
	source install/setup.bash && \
	ros2 launch launch/f1eighth.launch.yaml

clean:
	@while true; do \
		read -p 'Are you sure to clean up? (yes/no) ' yn; \
		case $$yn in \
			yes ) rm -rf build install log; break;; \
			no ) break;; \
			* ) echo 'Please enter yes or no.';; \
		esac \
	done
