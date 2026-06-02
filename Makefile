DOCKER_COMPOSE ?= docker compose
SERVICE ?= demo

.PHONY: shell build demo demo-llm demo-llm-directive

shell:
	$(DOCKER_COMPOSE) run --rm $(SERVICE) bash

build:
	$(DOCKER_COMPOSE) build $(SERVICE)

demo:
	$(DOCKER_COMPOSE) run --rm demo

demo-llm:
	$(DOCKER_COMPOSE) run --rm \
		-e LLM_MODEL_PATH=$(LLM_MODEL_PATH) \
		demo_llm
