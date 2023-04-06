# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import time
import carla
import multiprocessing as mp

from carla_simulation.infrastructure import Infrastructure
from carla_simulation.runner import Runner


class Balancer:

    _infrastructure = None

    def __init__(self, directory, jobs = 1, visualization = False):
        self._infrastructure = Infrastructure(
            jobs = jobs,
            scenarios = directory,
            visualization = visualization
        )

    def start(self):
        self._infrastructure.start()

    def stop(self):
        self._infrastructure.stop()

    def run(self):
        map_name = 'Town01'
        agent_name = 'FMIAdapter'
        metric_name = 'RawData'

        servers = self._infrastructure.get_servers()
        clients = self._infrastructure.get_clients()

        for server in servers:
            client = carla.Client(
                self._infrastructure.get_address(server),
                2000
            )
            server_map = client.get_world().get_map().name.split('/')[-1]
            if server_map != map_name:
                client.load_world(map_name)

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
                    agent_name,
                    metric_name
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
