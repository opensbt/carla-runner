# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import multiprocessing
import os
import subprocess
import threading
import concurrent.futures
import docker
import carla
import carla_simulation

import importlib.resources as pkg_resources

from time import sleep
from typing import Callable, List
from docker.models.containers import Container
from carla_simulation.utils import  map

def background_rosco_launch(client: Container) -> None:
    exec_result = client.exec_run(
        cmd='/bin/bash -c "{}"'.format(
            " && ".join([
                "source devel/setup.bash",
                "roslaunch rosco rosco.launch"
            ])
        ),
        workdir='/opt/workspace',
    )
    # Wait for the command to finish and retrieve the exit code
    exit_code = exec_result.exit_code

    # Only print when code is nonzero
    if exit_code != 0:
        # Retrieve the logs from the command
        logs = exec_result.output.decode('utf-8')

        # Print the exit code and logs
        print(f"\n[Infrastructure] Container {client.name} is unable to start rosco. Exit code {exit_code}\n"
              f"{logs}")


class Infrastructure:
    SERVER_IMAGE = 'carlasim/carla:0.9.15'
    CLIENT_IMAGE = 'carla-client'
    SERVER_PREFIX = 'carla-server'
    CLIENT_PREFIX = 'carla-client'

    NETWORK = 'bridge'

    RECORDINGS_DIR = '/tmp/recordings'
    SCENARIOS_DIR = '/tmp/scenarios'
    FAULTS_DIR = '/tmp/faults'

    MAP_NAME = 'Town01'

    CARLA_TIMEOUT = 20
    MAXIMUM_CONNECT_TRIES = 3

    POSSIBLE_QUALITY_LEVELS = ["Low", "Medium", "Epic"]

    def __init__(self,
        jobs = 1,
        scenarios_dir = SCENARIOS_DIR,
        recordings_dir = RECORDINGS_DIR,
        faults_dir = FAULTS_DIR,
        visualization = False,
        keep_carla_servers = False,
        rendering_quality="Medium"
        ):
        self.jobs = jobs
        self.network = self.NETWORK
        self.scenarios_dir = scenarios_dir
        self.recordings_dir = recordings_dir
        self.faults_dir = faults_dir
        self.client = docker.from_env()
        self.clients: List[Container] = []
        self.servers: List[Container] = []
        self.visualization = visualization
        self.keep_carla_servers = keep_carla_servers
        if rendering_quality in self.POSSIBLE_QUALITY_LEVELS:
            self.rendering_quality = rendering_quality
            print("[Infrastructure] Rendering quality was set to: " + self.rendering_quality)
        else:
            self.rendering_quality = "Medium"
            print("[Infrastructure] Requested simulation quality '" +  rendering_quality
                  + "' does not exist. Therefore, the default is used: '" + self.rendering_quality
                  + "'. Available levels: '" + "' - '".join(self.POSSIBLE_QUALITY_LEVELS) + "'.")
        print("If you have changed the simulation quality, please rebuild the Docker containers."
              + " Otherwise the infrastructure will not be affected.")

    def start(self):
        subprocess.run('xhost +local:root', shell=True)
        os.makedirs(self.recordings_dir, exist_ok=True)
        print("Getting images...", end="")

        # Ensure server image is pulled
        print(" Pulling...", end="")
        self.client.images.pull(self.SERVER_IMAGE)

        # Ensure client image is built
        print(" Building...", end="")
        build_args = {"TOKEN": os.environ.get("gitpass")}
        with pkg_resources.path(carla_simulation, 'Dockerfile') as file:
            path = str(file.resolve().parents[0])
            self.client.images.build(
                tag = self.CLIENT_IMAGE,
                path = path,
                buildargs = build_args
            )

        print(" Creating containers...")
        for job in range(self.jobs):
            server = self.create_server(
                id=job
            )
            self.servers.append(server)

            client = self.create_client(
                id=job
            )
            self.clients.append(client)

        print("Starting containers... ", end="")
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
        while not self.get_address(server):
            server.reload()
            sleep(1)
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
                version = new_client.get_server_version()
                # Successfully connected to server
                print(f" Server Version: {version}.", end="")
                carla_client = new_client

            except RuntimeError:
                # Catch exception and retry to a maximum of 5 times
                print(f".", end='')

                if tries >= self.MAXIMUM_CONNECT_TRIES:
                    print(" Giving up.")
                    raise RuntimeError("Cannot contact carla server. Is it running?")
            tries += 1

        # Set timeout larger to avoid timeout errors when the carla server is just slow to respond
        carla_client.set_timeout(self.CARLA_TIMEOUT)
        map.prepare(carla_client,scenario_map=self.MAP_NAME)

        print("Done")

    def configure_running_client(self, client: Container) -> None:
        while not self.get_address(client):
            client.reload()
            sleep(1)
        background_thread = threading.Thread(target=background_rosco_launch, args=(client,))

        # Start the thread
        background_thread.start()

    def get_servers(self) -> List[Container]:
        return self.servers

    def get_clients(self) -> List[Container]:
        return self.clients

    def stop(self):
        containers = self.servers + self.clients
        if containers:
            # Use more threads to stop the containers concurrently. These threads are only waiting and no processing
            # anything so ideally there are as many threads as containers
            with concurrent.futures.ThreadPoolExecutor(multiprocessing.cpu_count()) as executor:
                executor.map(self.stop_container, containers)
                containers.clear()

    def stop_container(self, container: Container) -> None:
        # if carla containers are kept, remove everything
        # if carla servers are kept, remove only containers which are not servers
        if not self.keep_carla_servers or container not in self.servers:
            try:
                print(f"Stopping container {container.name}.")
                container.stop()
            except docker.errors.NotFound:
                print(f"Container {container.name} does no longer exist.")

    def create_container_if_not_exist(self,
                                      container_name: str,
                                      container_factory: Callable[[], Container]) -> Container:
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
                restart_policy = {"Name": "on-failure", "MaximumRetryCount": 10},
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
                        self.scenarios_dir,
                        self.SCENARIOS_DIR),
                    '{}:{}:ro'.format(
                        self.faults_dir,
                        self.FAULTS_DIR),
                    '{}:{}:rw'.format(
                        self.recordings_dir,
                        self.RECORDINGS_DIR
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
                    '-quality-level=' + self.rendering_quality,
                ]
            )

        return self.create_container_if_not_exist(server_name, create_server_container)

    def create_client(self, id = None):
        client_name = self.CLIENT_PREFIX
        if id is not None:
            client_name += '-{}'.format(id)

        def create_client_container() -> Container:
            print(f"Creating client container {client_name}. ", end='')
            volume_mapping = [
                '/tmp/.X11-unix:/tmp/.X11-unix',
                    '/var/run/docker.sock:/var/run/docker.sock',
                    '{}:{}:ro'.format(
                        self.scenarios_dir,
                        self.SCENARIOS_DIR),
                    '{}:{}:ro'.format(
                        self.faults_dir,
                        self.FAULTS_DIR),
                    '{}:{}:rw'.format(
                        self.recordings_dir,
                        self.RECORDINGS_DIR
                        ),
                    '{SHARE_PATH}:/opt/workspace/share:rw'.format(
                        SHARE_PATH = os.environ['SHARE_PATH']
                        )
            ]
            if 'ROSCO_PATH' in os.environ:
                        volume_mapping.append('{ROSCO_PATH}:/opt/workspace/src/rosco:rw'.format(
                            ROSCO_PATH = os.environ['ROSCO_PATH']
                        ))
            if 'CARLA_PATH' in os.environ:
                        volume_mapping.append('{CARLA_PATH}:/opt/CARLA/Simulator:ro'.format(
                            CARLA_PATH = os.environ['CARLA_PATH']
                        ))
            if 'SCENARIORUNNER_PATH' in os.environ:
                        volume_mapping.append('{SCENARIORUNNER_PATH}:/opt/CARLA/Runner:ro'.format(
                            SCENARIORUNNER_PATH = os.environ['SCENARIORUNNER_PATH']
                        ))
            if 'OPENSBT_RUNNER_PATH' in os.environ:
                        volume_mapping.append('{OPENSBT_RUNNER_PATH}:/opt/OpenSBT/Runner:rw'.format(
                            OPENSBT_RUNNER_PATH = os.environ['OPENSBT_RUNNER_PATH']
                        ))
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
                        '/opt/CARLA/Simulator/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg',
                        '/opt/CARLA/Simulator/PythonAPI/carla/agents',
                        '/opt/CARLA/Simulator/PythonAPI/carla',
                        '/opt/CARLA/Runner'
                    ])),
                    'CARLA_ROOT=/opt/CARLA/Simulator',
                    'SCENARIO_RUNNER_ROOT=/opt/CARLA/Runner',
                ],
                volumes = volume_mapping,
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
                        "pip install --ignore-installed -r /opt/OpenSBT/Runner/requirements.txt",
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
                        "pip install --ignore-installed --force-reinstall /opt/OpenSBT/Runner/dist/*.whl",
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
