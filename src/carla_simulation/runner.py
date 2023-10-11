# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import json

from docker.models.containers import Container
from carla_simulation.infrastructure import Infrastructure


class Runner:
    _infrastructure: Infrastructure = None

    _server: Container = None
    _client: Container = None

    _agent_name: str = None
    _metric_name: str = None

    _fault = None

    MAX_RESTARTS = 3
    SUCCESS_INDICATOR = "[Executor] SUCCESS:"
    FAILURE_INDICATOR = "[Executor] ERROR:"

    def __init__(self,
                 infrastructure,
                 server: Container,
                 client: Container,
                 agent: str,
                 metric: str,
                 fault: str):
        self._infrastructure = infrastructure
        self._server = server
        self._client = client
        self._agent_name = agent
        self._metric_name = metric
        self._fault = fault

    def run(self, queue, evaluations):
        while not queue.empty():
            pattern = queue.get()
            success = False
            attempts = 0
            while not success:
                print(f"[Runner] Running Scenario {pattern}, Attempt {attempts}.")
                configuration = " ".join([
                    "--host {}".format(self._infrastructure.get_address(self._server)),
                    "--recordings {}".format(self._infrastructure.RECORDING_DIR),
                    "--scenarios {}".format(self._infrastructure.SCENARIO_DIR),
                    "--pattern {}".format(pattern),
                    "--agent {}".format(self._agent_name),
                    "--metric {}".format(self._metric_name),
                    "--faultInjection {}".format(self._infrastructure.FAULTS_DIR)
                ])
                if self._infrastructure.visualization:
                    configuration = "{} --visualize".format(configuration)

                _, stream = self._client.exec_run(
                    cmd='/bin/bash -c "{}"'.format(
                        " && ".join([
                            "source /opt/workspace/devel/setup.bash",
                            "python3.8 executor.py {}".format(configuration),
                        ])
                    ),
                    workdir='/opt/OpenSBT/Runner/src/carla_simulation',
                    stream=True
                )

                # Print log output generated by the command inside the container
                last_chars = ""
                for data in stream:
                    last_chars += data.decode()
                    last_chars = last_chars[-1000:]
                    print(data.decode(), end='')

                if self.FAILURE_INDICATOR in last_chars:
                    print(f"[Runner] Executor ran into an problem while in scenario {pattern}, agent {self._agent_name}.")
                    print("[Runner] Trying to start the carla server ...")
                    self._server.start()
                    self._infrastructure.configure_running_server(self._server)

                    attempts += 1
                    if attempts > self.MAX_RESTARTS:
                        # Maximum tries exceeded, aborting
                        break

                    # Continue, to run the scenario again
                    continue
                #elif self.SUCCESS_INDICATOR not in last_chars:
                    #print("WARNING: Executor feedback not found, assuming success.")
                print("[Runner] Done: {}".format(pattern))

                pattern = pattern.replace('xosc', 'json')
                with os.scandir(self._infrastructure.recordings) as entries:
                    for entry in entries:
                        if entry.name.endswith(pattern) and entry.is_file():
                            with open(entry, 'r') as file:
                                metrics = json.loads(file.read())
                                evaluations[pattern] = metrics
                            os.remove(entry)

                success = True
                # The scenario is complete and does not need to be executed again
                break
            #if not success:
            #    print(
            #        f"ERROR: Giving up on scenario {pattern}, agent {self._agent_name}. "
            #        f"Marking as complete without result.")
            queue.task_done()
