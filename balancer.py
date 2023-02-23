# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import time
import carla
import multiprocessing as mp

from infrastructure import Infrastructure
from runner import Runner

def run_scenarios(scenario_dir, visualization_flag = False):
    i = Infrastructure(
        jobs = 1,
        scenarios = scenario_dir,
    )
    i.start()

    map_name = 'Town01'
    agent_name = 'FMIAdapter'
    metric_name = 'RawData'

    servers = i.get_servers()
    clients = i.get_clients()

    for server in servers:
        client = carla.Client(i.get_address(server), 2000)
        server_map = client.get_world().get_map().name.split('/')[-1]
        if server_map != map_name:
            client.load_world(map_name)

    scenarios = mp.JoinableQueue()
    with os.scandir(scenario_dir) as entries:
        for entry in entries:
            if entry.name.endswith('.xosc') and entry.is_file():
                scenarios.put(entry.name)

    with mp.Manager() as manager:
        start_time = time.time()

        evaluations = manager.list()
        for server, client in zip(servers, clients):
            runner = Runner(i,
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

        stop_time = time.time()

        print('Time: {}s'.format(stop_time - start_time))

        i.stop()

        return list(evaluations)
