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
    
    _fault = None

    def __init__(self, infrastructure, server, client, agent, metric, fault):
        self._infrastructure = infrastructure
        self._server = server
        self._client = client
        self._agent_name = agent
        self._metric_name = metric
        self._fault = fault

    def run(self, queue, evaluations):
        print("hallo")
        while not queue.empty():
            pattern = queue.get()
            print("hallo2")

            configuration = " ".join([
                "--host {}".format(self._infrastructure.get_address(self._server)),
                "--recordings {}".format(self._infrastructure.RECORDING_DIR),
                "--scenarios {}".format(self._infrastructure.SCENARIO_DIR),
                "--pattern {}".format(pattern),
                "--agent {}".format(self._agent_name),
                "--metric {}".format(self._metric_name),
                "--fault {}".format(self._fault)
            ])
            print("hallo3")
            
            if (self._infrastructure.visualization):
                configuration = "{} --visualize".format(configuration)
            print("hallo4")
            print(configuration)

            _, stream =self._client.exec_run(
                cmd = '/bin/bash -c "{}"'.format(
                    " && ".join([
                        "source /opt/workspace/devel/setup.bash",
                        "python3.8 executor.py {}".format(configuration),
                    ])
                ),
                workdir = '/opt/OpenSBT/Runner/src/carla_simulation/',
                stream = True
            )
            print("hallo4.5")
            for data in stream:
                print(data.decode(), end=' ')
            print("hallo5")
            pattern = pattern.replace('xosc', 'json')
            with os.scandir(self._infrastructure.recordings) as entries:
                for entry in entries:
                    if entry.name.endswith(pattern) and entry.is_file():
                        with open(entry, 'r') as file:
                            obj = json.loads(file.read())
                            evaluations.append(obj)
                        os.remove(entry)

            queue.task_done()
