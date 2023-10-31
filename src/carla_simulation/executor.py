# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import sys
import json
import argparse
import traceback

from carla_simulation.simulator import Simulator
from carla_simulation.scenario import Scenario
from carla_simulation.recorder import Recorder

from carla_simulation.metrics.raw import RawData

from carla_simulation.controllers.fmi import FMIAgent
from carla_simulation.controllers.npc import NPCAgent

FAULT_DIR = "/tmp/faults"

class Executor:

    agents = {
        'FMIAgent': FMIAgent,
        'NPCAgent': NPCAgent
    }

    metrics = {
        'RawData': RawData
    }

    _host_carla = None
    _port_carla = 2000

    _timeout_carla = 10
    _rendering_carla = False
    _resolution_carla = 0.1
    _synchronous_carla = True

    _enable_manual_control = False

    _agent_class = None
    _metric_class = None

    _recording_dir = None
    _scenario_dir = None
    _fault_dir = None

    _faults = []

    def __init__(self, host, scenario_dir, recording_dir, agent, metric,
                 resolution, synchronous, visualize, enable_manual_control, faultInjection):
        self._host_carla = host

        self._recording_dir = recording_dir
        self._scenario_dir = scenario_dir

        self._agent_class = self.agents.get(agent)
        self._metric_class = self.metrics.get(metric)

        self._rendering_carla = visualize
        self._fault_dir = faultInjection

        self._resolution_carla = resolution
        self._synchronous_carla = synchronous

        self._enable_manual_control = enable_manual_control


    def execute(self, pattern):
        try:
            simulator = self.get_simulator(
                self._host_carla,
                self._port_carla,
                self._timeout_carla,
                self._rendering_carla,
                self._resolution_carla,
                self._synchronous_carla,
                self._enable_manual_control
            )

            scenarios = self.get_scenarios(self._scenario_dir, pattern)
            recorder = self.get_recorder(self._recording_dir)
            evaluator = self.get_evaluator()
            self.agents.get('FMIAgent').setFault(self._fault_dir+"/"+pattern)
            agent = self.agents.get('FMIAgent')

            for scenario in scenarios:
                scenario.simulate(simulator, agent, recorder)

            recordings = recorder.get_recordings()

            for recording in recordings:
                evaluation = evaluator.evaluate(
                    simulator,
                    recording
                )
                path = '{}.json'.format(os.path.splitext(recording)[0])
                with open(path, 'w') as file:
                    file.write(json.dumps(evaluation))
                os.remove(recording)
        except Exception:
            print(traceback.format_exc())
            print("[Executor] ERROR: Exception encountered.")
            sys.stdout.flush()
            os._exit(1)
        else:
            print("[Executor] SUCCESS: Completed all tasks.")

    def get_simulator(self, host, port, timeout, rendering = True,
                      resolution = 0.1, synchronous = True,
                      enable_manual_control = False):
        return Simulator(
            host = host,
            port = port,
            timeout = timeout,
            rendering = rendering,
            temporal_resolution = resolution,
            synchronous_execution = synchronous,
            enable_manual_control = enable_manual_control
        )

    def get_scenarios(self, directory, pattern):
        scenarios = None
        with os.scandir(directory) as entries:
            scenarios = [
                Scenario(entry)
                    for entry in entries
                        if entry.name.endswith(pattern) and entry.is_file()
            ]
        return scenarios

    def get_faults(self, directory, pattern):
        faults = None
        with os.scandir(directory) as entries:
            faults = [
                entry
                    for entry in entries
                        if entry.name.endswith(pattern) and entry.is_file()
            ]
        return faults

    def get_evaluator(self):
        return self._metric_class()

    def get_agent(self):
        return self._agent_class

    def get_recorder(self, directory):
        return Recorder(directory)

def main():
    parser = argparse.ArgumentParser(description='Execute a set of scenarios.')
    parser.add_argument(
        '--host',
        help='Hostname of the CARLA server.',
        required=True
    )
    parser.add_argument(
        '--scenarios',
        help='Directory containing all scenarios.',
        required=True
    )
    parser.add_argument(
        '--recordings',
        help='Directory to store the recordings.',
        required=True
    )
    parser.add_argument(
        '--agent',
        help='Agent to execute.',
        required=True
    )
    parser.add_argument(
        '--metric',
        help='Metric module to apply.',
        required=True
    )
    parser.add_argument(
        '--pattern',
        help='Pattern for selecting the scenarios.',
        required=True
        
    )
    parser.add_argument(
        '--resolution',
        help='The resolution of the simulation tick time.',
        default=None,
        required=False,
        type=float
    )
    parser.add_argument(
        '--synchronous',
        help='Whether the simulation should be synchronous (between server and client).',
        required=False,
        action='store_true'
    )
    parser.add_argument(
        '--visualize',
        help='Visualize the scenarios.',
        required=False,
        action='store_true'
    )
    parser.add_argument(
        '--enable_manual_control',
        help='Enable manual control of the vehicle with the keyboard during simulation.',
        required=False,
        action='store_true'
    )
    parser.add_argument(
        '--faultInjection',
        help='Name of faultinjection.',
        required=False
    )

    args = parser.parse_args()


    e = Executor(args.host, args.scenarios, args.recordings, args.agent,
                 args.metric, args.resolution, args.synchronous, args.visualize,
                 args.enable_manual_control, args.faultInjection)
    e.execute(args.pattern)

if __name__ == '__main__':
    sys.exit(main())
