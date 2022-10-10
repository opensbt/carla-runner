# Copyright (c) 2021 Universitat Autonoma de Barcelona (UAB)
# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from xmlrpc.client import Fault
import carla

from visualizations.real import CameraView

from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


class FMIAgent(AutonomousAgent):

    _agent = None
    _route_assigned = False
    _visual = None

    def __init__(self, simulator):
        super().__init__("")
        if not simulator.get_client().get_world().get_settings().no_rendering_mode:
            self._visual = CameraView('center')

    def setup(self, _):
        self._agent = None

    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)
        return carla.VehicleControl(throttle=1.0,
                                           steer=0.0,
                                           brake=0.0,
                                           hand_brake=False,
                                           reverse=False,
                                           manual_gear_shift=False,
                                           gear=1)

    def sensors(self):
        sensors = []
        if self._visual is not None:
            sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 0.7, 'y': 0.0, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'channels': 1, 'range': 10.0, 'points_per_second': 56000,
                    'rotation_frequency': 10.0, 'upper_fov': 1.0, 'lower_fov': 1.0,
                    'horizontal_fov': 1.0, 'id': 'center'
                }
            )
        return sensors

    def destroy(self):
        if self._visual is not None:
            self._visual.quit = True
