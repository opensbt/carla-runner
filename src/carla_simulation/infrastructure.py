# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
from typing import Callable, List

import carla
import docker
from docker.models.containers import Container
import subprocess


import carla_simulation

from time import sleep

import importlib.resources as pkg_resources

class Infrastructure:

    SERVER_IMAGE = 'carlasim/carla:0.9.13'
    CLIENT_IMAGE = 'carla-client'
    SERVER_PREFIX = 'carla-server'
    CLIENT_PREFIX = 'carla-client'

    NETWORK = 'bridge'

    RECORDING_DIR = '/tmp/recordings'
    SCENARIO_DIR = '/tmp/scenarios'

    MAP_NAME = 'Town01'
    MAXIMUM_CONNECT_TRIES = 3

    def __init__(self,
        jobs = 1,
        scenarios = SCENARIO_DIR,
        recordings = RECORDING_DIR,
        visualization = False,
        keep_carla_servers = False
        ):
        self.jobs = jobs
        self.network = self.NETWORK
        self.scenarios = scenarios
        self.recordings = recordings
        self.client = docker.from_env()
        self.clients: List[Container] = []
        self.servers: List[Container] = []
        self.visualization = visualization
        self.keep_carla_servers = keep_carla_servers

    def start(self):
        subprocess.run('xhost +local:root', shell=True)
        os.makedirs(self.recordings, exist_ok=True)
        print("Getting images... ", end="")
        # Ensure server image is pulled
        print(" Pulling...", end="")
        self.client.images.pull(self.SERVER_IMAGE)

        # Ensure client image is built
        print(" Building... ", end="")
        with pkg_resources.path(carla_simulation, 'Dockerfile') as file:
            path = str(file.resolve().parents[0])
            self.client.images.build(
                tag = self.CLIENT_IMAGE,
                path = path,
            )

        print(" Creating containers... ")
        for job in range(self.jobs):
            server = self.create_server(
                id=job
            )
            self.servers.append(server)

            client = self.create_client(
                id=job
            )
            self.clients.append(client)

        print("Starting containers ... ", end="")
        for client in self.clients:
            client.start()

        for server in self.servers:
            server.start()

        for client in self.clients:
            self.configure_running_client(client)

        for server in self.servers:
            self.configure_running_server(server)

        print("[Infrastructure] All up")

    def configure_running_server(self, server: Container):
        print(f"Connecting to {server.name}...", end='')
        carla_client = None
        tries = 0
        # Connect to carla server
        while carla_client is None:
            try:
                # Create client in each try
                new_client = carla.Client(
                    self.get_address(server),
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
        if server_map != self.MAP_NAME:
            print(f" Loading Map... ", end='')
            carla_client.load_world(self.MAP_NAME)
        else:
            # Map is already present, so we are not reloading to save time.
            # This means that actors from previous scenarios will stay on the map.
            # However, scenario.py will remove them, before loading new actors
            print(f" Map present. ", end='')

        print("Done")

    def configure_running_client(self, client: Container) -> None:
        while not self.get_address(client):
            client.reload()
            sleep(1)
        client.exec_run(
            cmd='/bin/bash -c "{}"'.format(
                " && ".join([
                    "source devel/setup.bash",
                    "roslaunch rosco rosco.launch"
                ])
            ),
            workdir='/opt/workspace',
            detach=True
        )

    def get_servers(self) -> List[Container]:
        return self.servers

    def get_clients(self) -> List[Container]:
        return self.clients

    def stop(self):
        containers = self.servers + self.clients
        if containers:
            for container in containers:
                if container in self.servers and self.keep_carla_servers:
                    continue
                try:
                    print(f"Stopping container {container.name}")
                    container.stop()
                except docker.errors.NotFound:
                    print(f"Container {container.name} does no longer exist")
            containers.clear()

    def create_container_if_not_exist(self, container_name: str, container_factory: Callable[[], Container]) -> Container:
        container = None
        try:
            container = self.client.containers.get(container_name)
            print(f"Found container {container_name}. Reusing.")

        except docker.errors.NotFound:
            container = container_factory()

        return container

    def create_server(self, id=None) -> Container:
        server_name = self.SERVER_PREFIX
        if id is not None:
            server_name += '-{}'.format(id)

        def create_server_container() -> Container:
            print(f"Creating server container {server_name}")
            return self.client.containers.run(
                self.SERVER_IMAGE,
                name = server_name,
                detach = True,
                privileged = True,
                remove = False,
                user = f"{os.getuid()}:{os.getgid()}",
                network_mode = self.network,
                environment = [
                    'DISPLAY={}'.format(
                        os.environ['DISPLAY']
                    )
                ],
                volumes = [
                    '{}:{}:ro'.format(
                        self.scenarios,
                        self.SCENARIO_DIR),
                    '{}:{}:rw'.format(
                        self.recordings,
                        self.RECORDING_DIR
                    ),
                ],
                device_requests = [
                    docker.types.DeviceRequest(
                        count=-1,
                        capabilities=[['gpu']]
                    )
                ],
                command = [
                    '/bin/bash',
                    './CarlaUE4.sh',
                    '-RenderOffScreen',
                    '-quality-level=Low',
                ]
            )
        return self.create_container_if_not_exist(server_name, create_server_container)

    def create_client(self, id = None):
        client_name = self.CLIENT_PREFIX
        if id is not None:
            client_name += '-{}'.format(id)

        def create_client_container() -> Container:
            print(f"Creating client container {client_name}. ", end='')
            container = self.client.containers.run(
                self.CLIENT_IMAGE,
                name = client_name,
                detach = True,
                privileged = True,
                remove = False,
                network_mode = self.network,
                ipc_mode = 'host',
                environment = [
                    'DISPLAY={DISPLAY}'.format(
                        DISPLAY = os.environ['DISPLAY']
                    ),
                    'PYTHONPATH={}'.format(':'.join([
                        # '/opt/OpenSBT/Runner/src',
                        '/opt/CARLA/Simulator/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg',
                        '/opt/CARLA/Simulator/PythonAPI/carla/agents',
                        '/opt/CARLA/Simulator/PythonAPI/carla',
                        '/opt/CARLA/Runner'
                    ])),
                    'CARLA_ROOT=/opt/CARLA/Simulator',
                    'SCENARIO_RUNNER_ROOT=/opt/CARLA/Runner',
                ],
                volumes = [
                    '/tmp/.X11-unix:/tmp/.X11-unix',
                    '/var/run/docker.sock:/var/run/docker.sock',
                    '{}:{}:ro'.format(
                        self.scenarios,
                        self.SCENARIO_DIR),
                    '{}:{}:rw'.format(
                        self.recordings,
                        self.RECORDING_DIR
                    ),
                    '{ROSCO_PATH}:/opt/workspace/src/rosco:rw'.format(
                        ROSCO_PATH = os.environ['ROSCO_PATH']
                    ),
                    '{SHARE_PATH}:/opt/workspace/share:rw'.format(
                        SHARE_PATH = os.environ['SHARE_PATH']
                    ),
                    '{CARLA_PATH}:/opt/CARLA/Simulator:ro'.format(
                        CARLA_PATH = os.environ['CARLA_PATH']
                    ),
                    '{SCENARIORUNNER_PATH}:/opt/CARLA/Runner:ro'.format(
                        SCENARIORUNNER_PATH = os.environ['SCENARIORUNNER_PATH']
                    ),
                    '{OPENSBT_CORE_PATH}:/opt/OpenSBT/Core:rw'.format(
                        OPENSBT_CORE_PATH = os.environ['OPENSBT_CORE_PATH']
                    ),
                    '{OPENSBT_RUNNER_PATH}:/opt/OpenSBT/Runner:rw'.format(
                        OPENSBT_RUNNER_PATH = os.environ['OPENSBT_RUNNER_PATH']
                    ),
                ],
                device_requests = [
                    docker.types.DeviceRequest(
                        count=-1,
                        capabilities=[['gpu']]
                    )
                ],
                command = 'sleep infinity'
            )

            while not self.get_address(container):
                container.reload()
                sleep(1)
            print("Installing OpenSBT requirements... ", end="")
            container.exec_run(
                cmd = '/bin/bash -c "{}"'.format(
                    " && ".join([
                        "pip install -r /opt/OpenSBT/Runner/requirements.txt",
                        "pip install -r /opt/OpenSBT/Core/requirements.txt",
                    ])
                ),
                environment = {
                    "SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL": True
                }
            )
            print("Installing OpenSBT wheel... ", end="")
            container.exec_run(
                cmd = '/bin/bash -c "{}"'.format(
                    " && ".join([
                        "python3.8 -m build",
                        "pip install --force-reinstall /opt/OpenSBT/Runner/dist/*.whl",
                    ])
                ),
                workdir = '/opt/OpenSBT/Runner'
            )
            print("Building ROS Workspace... ", end="")
            container.exec_run(
                cmd = '/bin/bash -c "{}"'.format(
                    " && ".join([
                        "source /opt/ros/melodic/setup.bash",
                        "catkin_make",
                    ])
                ),
                workdir = '/opt/workspace'
            )

            return container
        return self.create_container_if_not_exist(client_name, create_client_container)

    def get_address(self, container):
        address = container.attrs[
            'NetworkSettings'
        ][
            'Networks'
        ][
            self.network
        ][
            'IPAddress'
        ]
        return address
