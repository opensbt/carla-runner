# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla


class Simulator:

    _host = None
    _enable_manual_control = False

    def __init__(self, host, port, timeout, rendering = False, resolution = 0.1, synchronous = True, enable_manual_control = False):
        self._host = host
        self._enable_manual_control = enable_manual_control

        self.client = carla.Client(host, port)
        self.client.set_timeout(timeout)

        world = self.client.get_world()

        settings = world.get_settings()
        settings.no_rendering_mode = not rendering
        settings.synchronous_mode = synchronous
        settings.fixed_delta_seconds = resolution
        settings.substepping = True
        settings.max_substeps = 10
        settings.max_substep_delta_time = resolution / settings.max_substeps

        world.apply_settings(settings)
        print('[Simulator] Mode: Synchronous [{}], Temporal Resolution [{}],'\
              ' Maximal Substeps [{}], Substep Resolution [{}]'.format(
                  synchronous,
                  resolution,
                  settings.max_substeps,
                  settings.max_substep_delta_time))

        traffic_manager = self.client.get_trafficmanager(
            self.get_traffic_manager_port()
        )
        traffic_manager.set_synchronous_mode(synchronous)

    def get_traffic_manager_port(self):
        return 8000 + int(self._host.split('.')[-1])

    def get_client(self):
        return self.client

    def is_manual_control_enabled(self):
        return self._enable_manual_control
