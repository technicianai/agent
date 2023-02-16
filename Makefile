# This makefile helps with building and deploying the agent docker image.
# This should be used in a deployment build, not really for local development where docker-compose --build
# is probably a better option.
# Inspired by https://github.com/flowerinthenight/golang-monorepo/blob/master/cmd/samplecmd/Makefile
VERSION ?= $(shell git describe --match=NeVeRmAtCh --always --abbrev=8 --dirty)
BLDVER = module:$(MODULE),version:$(VERSION),build:$(shell date +"%Y%m%d.%H%M%S.%N.%z")
MODULE = agent
ROOTDIR=${PWD}

.PHONY: all $(MODULE)
all: version $(MODULE)

# $(ROOTDIR):
	# @mkdir -p $(dir $@)

.PHONY: $(MODULE)
$(MODULE):| $(ROOTDIR)
	echo $(ROOTDIR)

.PHONY: custom docker deploy

install:
	docker buildx install
	docker buildx create --name=woeden --driver=docker-container --use --bootstrap

# The rule that is called by our root Makefile during CI builds.
custom: docker deploy

ECR_REPOSITORY=${ACCOUNT_ID}.dkr.ecr.${AWS_REGION}.amazonaws.com/${MODULE}
docker:
	docker buildx build \
		--builder=woeden \
		--platform=linux/arm64,linux/amd64 \
		-t $(MODULE):$(VERSION) \
	  -f $(ROOTDIR)/Dockerfile $(ROOTDIR)

deploy: docker
	@chmod +x $(ROOTDIR)/deploy.sh
	MODULE=$(MODULE) VERSION=$(VERSION) $(ROOTDIR)/deploy.sh

.PHONY: clean version list
clean:
	@rm -rfv bin
	@docker rmi $(docker images --filter "dangling=true" -q --no-trunc)
	@exit 0

version:
	@echo "Version: $(VERSION)"

list:
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | egrep -v -e '^[^[:alnum:]]' -e '^$@$$' | xargs

