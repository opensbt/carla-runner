# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import json
import time
from docker.models.containers import Container

from carla_simulation.infrastructure import Infrastructure


class Runner:
    _infrastructure: Infrastructure = None

    _server: Container = None
    _client: Container = None

    _agent_name: str = None
    _metric_name = None

    def __init__(self, infrastructure, server: Container, client: Container, agent: str, metric):
        self._infrastructure = infrastructure
        self._server = server
        self._client = client
        self._agent_name = agent
        self._metric_name = metric

    def run(self, queue, evaluations):
        while not queue.empty():
            pattern = queue.get()
            success = False
            for i in range(0, 2):
                print(f"[Runner] Running Scenario {pattern}, Attempt {i}")
                configuration = " ".join([
                    "--host {}".format(self._infrastructure.get_address(self._server)),
                    "--recordings {}".format(self._infrastructure.RECORDING_DIR),
                    "--scenarios {}".format(self._infrastructure.SCENARIO_DIR),
                    "--pattern {}".format(pattern),
                    "--agent {}".format(self._agent_name),
                    "--metric {}".format(self._metric_name)
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

                last_chars = ""
                # Print log output generated by the command inside the container
                for data in stream:
                    last_chars += data.decode()
                    last_chars = last_chars[-1000:]
                    print(data.decode(), end='')

                if "[Executor] ERROR:" in last_chars:
                    print(f"[Runner] Executor ran into an problem while in scenario {pattern}, agent {self._agent_name}")
                    print("[Runner] Trying to start the carla server")
                    self._server.start()
                    self._infrastructure.configure_running_server(self._server)

                    # Continue, to run the scenario again
                    continue
                elif "[Executor] SUCCESS:" not in last_chars:
                    print("WARNING: Executor feedback not found, assuming success")

                pattern = pattern.replace('xosc', 'json')
                with os.scandir(self._infrastructure.recordings) as entries:
                    for entry in entries:
                        if entry.name.endswith(pattern) and entry.is_file():
                            with open(entry, 'r') as file:
                                obj = json.loads(file.read())
                                evaluations.append(obj)
                            os.remove(entry)
                success = True
                # Otherwise the code would be executed again, so we break because our scenario is complete
                break
            if not success:
                print(
                    f"ERROR: Giving up on scenario {pattern}, agent {self._agent_name}, marking as complete without "
                    f"result")
            queue.task_done()
