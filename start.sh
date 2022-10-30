#!/bin/bash

echo "Building and executing rosco"
catkin_make
source devel/setup.bash
roslaunch rosco rosco.launch

echo "Executing balancer.py"
cd ../ff1_carla
export CARLA_ROOT="/opt/CARLA"
export PYTHONPATH="/opt/CARLA/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:/opt/CARLA/PythonAPI/carla/agents:/opt/CARLA/PythonAPI/carla:/opt/scenario_runner"
export SCENARIO_RUNNER_ROOT="/opt/scenario_runner"
python3.8 balancer.py