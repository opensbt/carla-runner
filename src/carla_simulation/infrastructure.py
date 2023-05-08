# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import docker
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

    def __init__(self,
        jobs = 1,
        scenarios = SCENARIO_DIR,
        recordings = RECORDING_DIR,
        visualization = False
    ):
        self.jobs = jobs
        self.network = self.NETWORK
        self.scenarios = scenarios
        self.recordings = recordings
        self.client = docker.from_env()
        self.clients = []
        self.servers = []
        self.visualization = visualization

    def start(self):
        subprocess.run('xhost +local:root', shell=True)
        os.makedirs(self.recordings, exist_ok=True)

        for job in range(self.jobs):
            server = self.create_server(
                id = job
            )
            self.servers.append(server)

            client = self.create_client(
                id = job
            )
            self.clients.append(client)

    def get_servers(self):
        return self.servers

    def get_clients(self):
        return self.clients

    def stop(self):
        containers = self.servers + self.clients
        if containers:
            for container in containers:
                try:
                    print(f"Stopping container {container.name}")
                    container.stop()
                except docker.errors.NotFound:
                    print(f"Container {container.name} does no longer exist")
            containers.clear()

    def create_server(self, id = None):
        server_name = self.SERVER_PREFIX
        if id is not None:
            server_name += '-{}'.format(id)

        self.client.images.pull(self.SERVER_IMAGE)

        container = None
        try:
            container = self.client.containers.get(server_name)
            print(f"Found server container {server_name}. Reusing.")
            container.start()

        except docker.errors.NotFound:
            print(f"Creating server container {server_name}")
            container = self.client.containers.run(
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

        while not self.get_address(container):
            container.reload()
            sleep(1)

        return container

    def create_client(self, id = None):
        client_name = self.CLIENT_PREFIX
        if id is not None:
            client_name += '-{}'.format(id)

        with pkg_resources.path(carla_simulation, 'Dockerfile') as file:
            path = str(file.resolve().parents[0])
            self.client.images.build(
                tag = self.CLIENT_IMAGE,
                path = path,
            )
        container = None
        try:
            container = self.client.containers.get(client_name)
            print(f"Found client container {client_name}. Reusing... ", end="")
            container.start()

        except docker.errors.NotFound:
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
                        '/opt/OpenSBT/Runner/src',
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
        container.exec_run(
            cmd = '/bin/bash -c "{}"'.format(
                " && ".join([
                    "python3.8 -m build",
                    "pip install --force-reinstall /opt/OpenSBT/Runner/dist/*.whl",
                ])
            ),
            workdir = '/opt/OpenSBT/Runner'
        )
        container.exec_run(
            cmd = '/bin/bash -c "{}"'.format(
                " && ".join([
                    "source /opt/ros/melodic/setup.bash",
                    "catkin_make",
                ])
            ),
            workdir = '/opt/workspace'
        )
        container.exec_run(
            cmd = '/bin/bash -c "{}"'.format(
                " && ".join([
                    "source devel/setup.bash",
                    "roslaunch rosco rosco.launch"
                ])
            ),
            workdir = '/opt/workspace',
            detach = True
        )

        return container

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
