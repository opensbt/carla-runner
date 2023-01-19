#!/bin/bash

cd /opt/search-based-test-case-generation

echo "Executing run.py"
export CARLA_ROOT="/opt/CARLA"
export PYTHONPATH="/opt/CARLA/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/opt/CARLA/PythonAPI/carla/agents:/opt/CARLA/PythonAPI/carla:/opt/scenario_runner:/opt/ff1_carla"
export SCENARIO_RUNNER_ROOT="/opt/scenario_runner"

source ../workspace/devel/setup.bash

python3.8 run.py -e 1 -i 3 -n 3