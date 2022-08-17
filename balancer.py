# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import queue
import threading

import docker
import matplotlib.pyplot as plt

from runner import Runner

from metrics.raw import RawData
from controllers.npc import NpcAgent


NETWORK_NAME = 'carla-network'
SCENARIO_DIR = 'scenarios'

client = docker.from_env()
network = client.networks.get(NETWORK_NAME)
servers = [
    container.attrs['NetworkSettings']['Networks'][NETWORK_NAME]['IPAddress']
    for container in network.containers
]

scenarios = queue.Queue()
with os.scandir(SCENARIO_DIR) as entries:
    for entry in entries:
        if entry.name.endswith('.xosc') and entry.is_file():
            scenarios.put(entry.name)

evaluations = list()
for server in servers:
    runner = Runner(
        server,
        NpcAgent,
        RawData
    )
    threading.Thread(
        target=runner.run,
        args=(SCENARIO_DIR, scenarios, evaluations),
        daemon = True
    ).start()

scenarios.join()

for evaluation in evaluations:
    time = evaluation["times"]
    distance = evaluation["velocity"]["ego"]
    plt.plot(time, distance)
    plt.ylabel('Velocity [m/s]')
    plt.xlabel('Time [s]')
    plt.title('Velocity')
    plt.show()
