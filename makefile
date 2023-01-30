SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PROJECT=roots-co-tech
USERNAME=$(whoami)

APP_NAME=ydlidar
DOCKER_REPO=us-east4-docker.pkg.dev/roots-co-tech/docker
TAG=latest
DEBUG=false
PUSH=true

install: update-go-deps build_go_application build_and_push_docker

build_go_application:
	CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build -mod vendor -a -o main

build_and_push_docker:
	docker build -t $(DOCKER_REPO)/$(APP_NAME):$(TAG) .
#	docker push $(DOCKER_REPO)/$(APP_NAME):$(TAG)

update-go-deps:
	@echo ">> updating Go dependencies"
	@for m in $$(go list -mod=readonly -m -f '{{ if and (not .Indirect) (not .Main)}}{{.Path}}{{end}}' all); do \
		go get $$m; \
	done
	go mod tidy
ifneq (,$(wildcard vendor))
	go mod vendor
endif
