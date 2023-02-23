# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import json

class Runner:

    _infrastructure = None

    _server = None
    _client = None

    _agent_name = None
    _metric_name = None

    def __init__(self, infrastructure, server, client, agent, metric):
        self._infrastructure = infrastructure
        self._server = server
        self._client = client
        self._agent_name = agent
        self._metric_name = metric

    def run(self, queue, evaluations):
        while not queue.empty():
            pattern = queue.get()

            self._client.exec_run(
                cmd = '/bin/bash -c "{}"'.format(
                    " && ".join([
                        "source /opt/workspace/devel/setup.bash",
                        "python3.8 executor.py {}".format(
                            " ".join([
                                "--host {}".format(self._infrastructure.get_address(self._server)),
                                "--recordings {}".format(self._infrastructure.RECORDING_DIR),
                                "--scenarios {}".format(self._infrastructure.SCENARIO_DIR),
                                "--pattern {}".format(pattern),
                                "--agent {}".format(self._agent_name),
                                "--metric {}".format(self._metric_name),
                            ])
                        ),
                    ])
                ),
                workdir = '/opt/OpenSBT/Runner'
            )

            with os.scandir(self._infrastructure.recordings) as entries:
                for entry in entries:
                    if entry.name.endswith('.json') and entry.is_file():
                        with open(entry, 'r') as file:
                            obj = json.loads(file.read())
                            evaluations.append(obj)

            queue.task_done()
