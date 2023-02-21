# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import docker
import subprocess

from time import sleep

class Infrastructure:

    SERVER_IMAGE = 'carlasim/carla:0.9.13'
    CLIENT_IMAGE = 'carla-client'
    SERVER_PREFIX = 'carla-server'
    CLIENT_PREFIX = 'carla-client'
    NETWORK_NAME = 'carla-network'

    def __init__(self):
        self.client = docker.from_env()
        self.network = None
        self.clients = []
        self.servers = []

    def start(self):
        subprocess.run('xhost +local:root', shell=True)
        os.makedirs('/tmp/recordings', exist_ok=True)

        network = self.client.networks.create(
            self.NETWORK_NAME
        )
        self.network = network

        server = self.create_server(
            id = 1
        )
        self.servers.append(server)

        client = self.create_client(
            id = 1
        )
        self.clients.append(client)

    def get_addresses(self):
        addresses = [
            self._get_address(container)
            for container in self.servers
        ]
        return addresses

    def stop(self):
        containers = self.servers + self.clients
        if containers:
            for container in containers:
                container.stop()
            containers.clear()

        if self.network is not None:
            self.network.remove()
            self.network = None

    def create_server(self, id = None):
        server_name = self.SERVER_PREFIX
        if id is not None:
            server_name += '-{}'.format(id)

        self.client.images.pull(self.SERVER_IMAGE)

        container = self.client.containers.run(
            self.SERVER_IMAGE,
            name = server_name,
            detach = True,
            privileged = True,
            remove = True,
            user = '1001:1001',
            network = self.NETWORK_NAME,
            environment = [
                'DISPLAY={}'.format(
                    os.environ['DISPLAY']
                )
            ],
            volumes = [
                '/tmp/recordings:/tmp/recordings:rw',
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

        while not self._get_address(container):
            container.reload()
            sleep(1)

        return container

    def create_client(self, id = None):
        client_name = self.CLIENT_PREFIX
        if id is not None:
            client_name += '-{}'.format(id)

        self.client.images.build(
            tag = self.CLIENT_IMAGE,
            path = './',
        )

        container = self.client.containers.run(
            self.CLIENT_IMAGE,
            name = client_name,
            detach = True,
            privileged = True,
            remove = True,
            network = 'host',
            ipc_mode = 'host',
            environment = [
                'DISPLAY={DISPLAY}'.format(
                    DISPLAY = os.environ['DISPLAY']
                ),
                'PYTHONPATH={}'.format(':'.join([
                    '/opt/OpenSBT/Runner',
                    '/opt/CARLA/Simulator/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg',
                    '/opt/CARLA/Simulator/PythonAPI/carla/agents',
                    '/opt/CARLA/Simulator/PythonAPI/carla',
                    '/opt/CARLA/Runner'
                ])),
                'CARLA_ROOT=/opt/CARLA/Simulator',
                'SCENARIO_RUNNER_ROOT=/opt/CARLA/Runner',
            ],
            volumes = [
                '/var/run/docker.sock:/var/run/docker.sock',
                '/tmp/recordings:/tmp/recordings:rw',
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

    def _get_address(self, container):
        address = container.attrs[
            'NetworkSettings'
        ][
            'Networks'
        ][
            self.NETWORK_NAME
        ][
            'IPAddress'
        ]
        return address
