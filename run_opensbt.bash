#!/bin/bash

cd /opt/OpenSBT/Core
source /opt/workspace/devel/setup.bash
echo "[OpenSBT] Running ..."
python3.8 run.py -e 1 -i 5 -n 5
