out-docker=out-docker
tag=pdm4ar


all:
	@echo "You can try:"
	@echo
	@echo "  make build"
	@echo "  make run-test"


##### Build commands
build:
	docker build -t $(tag) .

build-no-cache:
	docker build --no-cache -t $(tag) .


##### Run commands
define run_command
	mkdir -p $(out-docker)
    docker run --rm -v ${PWD}/$(out-docker):/$(out-docker) $(tag) pdm4ar-exercise \
    	-o /$(out-docker)/$(1) --reset -c "rparmake" --exercises $(1)
endef

define build_run_command
	$(MAKE) build
	$(call run_command,$(1))
endef

# Just testing the setup
run-test:
	$(call build_run_command,test)
# Lexicographic comparison
run-exercise1:
	$(call build_run_command,exercise1)
# Graph search
run-exercise2:
	$(call build_run_command,exercise2)
# Informed graph search
run-exercise3:
	$(call build_run_command,exercise3)
# Dynamic Programming Exercise
run-exercise4:
	$(call build_run_command,exercise4)
# Geometry poses
run-exercise5: |
	docker build -f Dockerfile.notebook -t "$(tag)_notebooks"  .
	docker run -p 8888:8888 --rm -it -v $(PWD):$(PWD) -w $(PWD) -e USER=$(USER) -v /tmp:/tmp -e HOME=/tmp/fake --user $(shell id -u):$(shell id -g) $(tag)_notebooks
run-final21:
	$(call build_run_command,final21)


#run-custom:
#	$(call run_command,"exercise1;test")
# Not supprted for now


##### to get the new exercises run "make update"
set-upstream: # Run this only once to set the upstream
	git remote add upstream git@github.com:idsc-frazzoli/PDM4AR-exercises.git

update:
	git pull upstream master


#### run skipping build (Mainly used in ci)
run-exercise0-nobuild:
	$(call run_command,test)
run-exercise1-nobuild:
	$(call run_command,exercise1)
run-exercise2-nobuild:
	$(call run_command,exercise2)
run-exercise3-nobuild:
	$(call run_command,exercise3)
run-exercise4-nobuild:
	$(call run_command,exercise4)
