# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os

import matplotlib.pyplot as plt

from simulator import Simulator
from scenario import Scenario
from recorder import Recorder
from metrics.raw import RawData
from controllers.npc import NpcAgent


HOST_CARLA = 'localhost'
PORT_CARLA = 2000
TIMEOUT_CARLA = 10
RENDERING_CARLA = True
RESOLUTION_CARLA = 0.1

RECORDING_DIR = '/tmp/recordings'
SCENARIO_DIR = 'scenarios'
METRICS_DIR = 'metrics'

def get_simulator(host, port, timeout, rendering = True, resolution = 0.1):
    return Simulator(
        host = host,
        port = port,
        timeout = timeout,
        rendering = rendering,
        resolution = resolution
    )

def get_scenarios(directory):
    scenarios = None
    with os.scandir(directory) as entries:
        scenarios = [
            Scenario(entry)
                for entry in entries
                    if entry.name.endswith('.xosc') and entry.is_file()
        ]
    return scenarios

def get_evaluator():
    return RawData()

def get_agent():
    return NpcAgent

def get_recorder(directory):
    return Recorder(directory)

simulator = get_simulator(
    HOST_CARLA,
    PORT_CARLA,
    TIMEOUT_CARLA,
    RENDERING_CARLA,
    RESOLUTION_CARLA
)
scenarios = get_scenarios(SCENARIO_DIR)
recorder = get_recorder(RECORDING_DIR)
evaluator = get_evaluator()
agent = get_agent()

for scenario in scenarios:
    scenario.simulate(simulator, agent, recorder)

recordings = recorder.get_recordings()

evaluations = list()
for recording in recordings:
    evaluations.append(
        evaluator.evaluate(
            simulator,
            recording
        )
    )

for evaluation in evaluations:
    time = evaluation["times"]
    distance = evaluation["velocity"]["ego"]
    plt.plot(time, distance)
    plt.ylabel('Velocity [m/s]')
    plt.xlabel('Time [s]')
    plt.title('Velocity')
    plt.show()
