#!/bin/bash

cd /opt/workspace
echo "Building and executing rosco"
catkin_make
source devel/setup.bash
roslaunch rosco rosco.launch