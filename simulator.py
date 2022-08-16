# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla


class Simulator:

    def __init__(self, host, port, timeout, resolution = 0.1, rendering=False):
        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)

        world = self.client.get_world()
        settings = world.get_settings()

        settings = world.get_settings()
        settings.no_rendering_mode = not rendering
        world.apply_settings(settings)

        settings.fixed_delta_seconds = resolution
        settings.synchronous_mode = True

        world.apply_settings(settings)

        traffic_manager = self.client.get_trafficmanager(int(8000))
        traffic_manager.set_synchronous_mode(True)

    def get_client(self):
        return self.client
