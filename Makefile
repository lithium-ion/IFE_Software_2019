DOCKER_IMAGE=ubuntu:16.04-ife-$(USER)

all: Dockerfile
	docker build -t $(DOCKER_IMAGE) .

add_user:
	sudo usermod -a -G docker $(USER)

run:
	docker run --privileged --rm -it -v "$(HOME):$(HOME)"--user="$(USER)" $(DOCKER_IMAGE)
