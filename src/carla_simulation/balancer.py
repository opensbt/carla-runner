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
    MAXIMUM_CONNECT_TRIES = 5

    def __init__(self, directory, jobs = 1, visualization = False, keep_carla_servers=False):
        self._infrastructure = Infrastructure(
            jobs = jobs,
            scenarios = directory,
            visualization = visualization,
            keep_carla_servers= keep_carla_servers
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
            print(f"Connecting to {server.name}...", end = '')
            carla_client = None
            tries = 0
            # Connect to carla server
            while carla_client is None:
                try:
                    # Create client in each try
                    new_client = carla.Client(
                        self._infrastructure.get_address(server),
                        2000
                    )
                    # Check server version
                    version = carla_client.get_server_version()
                    # Successfully connected to server
                    print(f" Server Version: {version}.", end="")
                    carla_client = new_client

                except RuntimeError:
                    # Catch exception and retry to a maximum of 5 times
                    print(f".", end='')

                    if tries >= self.MAXIMUM_CONNECT_TRIES:
                        print("Giving up")
                        raise RuntimeError("Cannot contact carla server, is it running?")
                tries += 1

            server_map = carla_client.get_world().get_map().name.split('/')[-1]

            # Check if map is already loaded
            if server_map != map_name:
                print(f" Loading Map... ", end = '')
                carla_client.load_world(map_name)
            else:
                # Map is already present, so we are not reloading to save time.
                # This means that actors from previous scenarios will stay on the map.
                # However, scenario.py will remove them, before loading new actors
                print(f" Map present. ", end='')

            print("Done")

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
