#!/bin/bash

pip install -r /opt/OpenSBT/Runner/requirements.txt
pip install -r /opt/OpenSBT/Core/requirements.txt

cd /opt/workspace
echo "[ROSCo] Building ..."
catkin_make
echo "[ROSCo] Running ..."
source devel/setup.bash
roslaunch rosco rosco.launch &

cd /opt/OpenSBT/Core
echo "[OpenSBT] Running ..."
python3.8 run.py -e 1 -i 3 -n 3
