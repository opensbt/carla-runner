# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import time
import multiprocessing as mp

from carla_simulation.infrastructure import Infrastructure
from carla_simulation.runner import Runner

class Balancer:

    _infrastructure = None
    _agent_name = None
    _metric_name = None
    _resolution : float = 0.1
    _synchronous : bool = True
    _enable_manual_control : bool = False

    def __init__(self, directory, agent, metric = 'RawData', jobs = 1, visualization = False, keep_carla_servers=False, resolution = 0.1, synchronous = True, enable_manual_control = False, quality = "Medium"):
        self._infrastructure = Infrastructure(
            jobs = jobs,
            scenarios = directory,
            visualization = visualization,
            keep_carla_servers= keep_carla_servers,
            quality = quality
        )
        self._agent_name = agent
        self._metric_name = metric
        self._resolution = resolution
        self._synchronous = synchronous
        self._enable_manual_control = enable_manual_control

    def start(self):
        self._infrastructure.start()

    def stop(self):
        self._infrastructure.stop()

    def run(self):
        servers = self._infrastructure.get_servers()
        clients = self._infrastructure.get_clients()

        scenarios = mp.JoinableQueue()
        with os.scandir(self._infrastructure.scenarios) as entries:
            for entry in entries:
                if entry.name.endswith('.xosc') and entry.is_file():
                    scenarios.put(entry.name)

        with mp.Manager() as manager:
            start_time = time.time()

            evaluations = manager.list()
            for server, client in zip(servers, clients):
                runner = Runner(self._infrastructure,
                    server,
                    client,
                    self._agent_name,
                    self._metric_name,
                    self._resolution,
                    self._synchronous,
                    self._enable_manual_control
                )
                mp.Process(
                    target=runner.run,
                    args=(scenarios, evaluations),
                    daemon = True
                ).start()

            scenarios.join()
            scenarios.close()

            stop_time = time.time()

            print('Time: {}s'.format(stop_time - start_time))

            return list(evaluations)
