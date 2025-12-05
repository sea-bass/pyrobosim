# PyRoboSim

[![PyRoboSim Tests](https://github.com/sea-bass/pyrobosim/actions/workflows/tests.yml/badge.svg?branch=main)](https://github.com/sea-bass/pyrobosim/actions/workflows/tests.yml)
[![Documentation Status](https://readthedocs.org/projects/pyrobosim/badge/?version=latest)](https://pyrobosim.readthedocs.io/en/latest/?badge=latest)
![Coverage Status](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/sea-bass/3761a8aa05af7b0e8c84210b9d103df8/raw/pyrobosim-test-coverage.json)

ROS 2 enabled 2D mobile robot simulator for behavior prototyping.

By Sebastian Castro, 2022-2025

Refer to the [full documentation](https://pyrobosim.readthedocs.io/) for setup, usage, and other concepts.

We look forward to your open-source contributions to PyRoboSim.
For more information, refer to the [contributor guide](CONTRIBUTING.md).

![Example animation of the simulator](docs/source/media/pyrobosim_demo.gif)

## Docker quickstart

PyRoboSim includes a Docker Compose workspace that mirrors the development setup documented on Read the Docs.

- Build the image you want to use (the default service is `demo`): `make build`.
- Run an interactive shell inside the container: `make shell`.
- Launch the basic demo directly: `make demo`.

## Local LLM demo

PyRoboSim can run a lightweight, fully local language model policy through the `demo_llm` Compose service. The service mounts a host directory that contains your model checkpoint:

- `models/` – contains a [GGUF](https://github.com/ggerganov/ggml) file that llama.cpp can load.
  LLM dependencies (including `llama-cpp-python`) are installed directly during the Docker build, so no local wheels are required.

### Install models

1. Create the directory that will be mounted into the container:
   ```bash
   mkdir -p models
   ```
2. Download or convert an LLM checkpoint to GGUF format (for example, `Mistral-7B-Instruct-v0.3.f16.gguf` or  `Qwen2.5-VL-7B-Instruct-Q8_0.gguf`) and place it inside `models/`. Rename it to `model.gguf` if you prefer to use the default path.

### Run the LLM demo

After the model is available locally you can start the demo with:

```bash
make demo-llm
```

If you kept the GGUF filename instead of renaming it to `model.gguf`, pass the container path using the `LLM_MODEL_PATH` variable when invoking make:

```bash
make demo-llm LLM_MODEL_PATH=/models/Mistral-7B-Instruct-v0.3.f16.gguf
```

The GUI exposes an “LLM directive” text box and `LLM Step` button so you can send natural-language goals to the robot. Use `--llm-auto-step` (already set by the Docker command) to watch the policy execute each directive automatically.

![LLM-directed mission generation demo](docs/source/media/llm.gif)
