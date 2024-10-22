.PHONY: default prepare build save

IMAGE_NAME = f1eighth

default:
	@echo 'Usage:'
	@echo
	@echo 'make prepare'
	@echo '    Install dependent pakcages and configure the'
	@echo '    environment to build the container image.'
	@echo
	@echo 'make build'
	@echo '    Build the container image.'
	@echo
	@echo 'make run'
	@echo '    Enter the container shell.'
	@echo
	@echo 'make save'
	@echo '    Save a Docker image file.'

prepare:
	sudo apt-get install qemu binfmt-support qemu-user-static
	docker run --rm --privileged multiarch/qemu-user-static --reset -p yes --credential yes
	sudo systemctl restart docker.service

build:
	# ./scripts/resolve_deps.sh
	docker build . \
		-t $(IMAGE_NAME) #\
		--platform linux/arm64

run: build
	dockre run \
		-it --rm \
		--net=host \
		--runtime nvidia \
		-e DISPLAY=$$DISPLAY \
		-v /tmp/.X11-unix/:/tmp/.X11-unix \
		-v $$PWD/..:/home/jetson/F1EIGHTH \
		$(IMAGE_NAME)

save: build
	docker save $(IMAGE_NAME) | zstd -T0 -o $(IMAGE_NAME).tar.zstd
