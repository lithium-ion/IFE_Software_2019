DOCKER_IMAGE=ubuntu:16.04-ife-$(USER)

all: Dockerfile
	docker build --build-arg user=$(USER) -t $(DOCKER_IMAGE) .

add_user:
	sudo usermod -a -G docker $(USER)

run:
	docker run --privileged --rm -it -v "$(HOME):$(HOME)" -v /dev/bus/usb/:/dev/bus/usb/ --user="$(USER)"   $(DOCKER_IMAGE)
