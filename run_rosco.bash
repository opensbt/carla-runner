#!/bin/bash

cd /opt/workspace
echo "[ROSCo] Building ..."
catkin_make
echo "[ROSCo] Running ..."
source devel/setup.bash
roslaunch rosco rosco.launch
