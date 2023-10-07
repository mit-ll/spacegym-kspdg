PYTHON = python3
PIP = pip3

.DEFAULT_GOAL = run

build:
	@bash scripts/build_docker.bash

run:
	@bash scripts/run.bash $(filter-out $@, $(MAKECMDGOALS))

# build and run
runblt: build lint test
	@bash scripts/run.bash $(filter-out $@, $(MAKECMDGOALS))

stop:
	@docker container kill $$(docker ps -q)

jupyter:
	@bash scripts/jupyter.bash

lint:
	@bash scripts/lint.bash
	@echo "✅✅✅✅✅ Lint is good! ✅✅✅✅✅"

test:
	@bash scripts/test.bash

benchmark:
	@bash scripts/benchmark.bash

mlflow:
	@bash scripts/mlflow.bash