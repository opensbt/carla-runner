# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy
from numpy.linalg import norm

from rosco.srv import *
from rosco.msg import *
from carla_simulation.visualizations.real import CameraView
from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

class FMIAgent(AutonomousAgent):

    _agent = None
    _route_assigned = False
    _visual = None
    _do_step_service = None
    _speed = None
    _previous_speed = [0.5, 0.5, 0.5]
    _ego = None

    def __init__(self, simulator, ego_vehicle: carla.Vehicle):
        super().__init__("")
        self.ego_vehicle = ego_vehicle
        if not simulator.get_client().get_world().get_settings().no_rendering_mode:
            self._visual = CameraView('center')

    def setup(self, _):
        self._agent = None

        rospy.init_node('fmi_agent')

        # Initialize and call the masterInit service.
        rospy.wait_for_service('master/init')
        init_master = rospy.ServiceProxy('master/init', MasterInitService)
        print('calling init/master service')
        init_master('/opt/workspace/src/rosco/share/config.json')

        # Initialize the doStepsUntil service.
        rospy.wait_for_service('master/doStepsUntil')
        self._do_step_service = rospy.ServiceProxy('master/doStepsUntil', DoStepsUntilService, persistent=True)
        print('initialized do step service')

    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)

        signals = SignalsMessage()

        # distance_to_front
        minDistance = 2.0
        for point in input_data['lidar'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp

        signals.floatSignals.append(FloatSignal("DistanceToFrontLaser", minDistance))

        # distance_to_front_left
        minDistance = 2.0
        for point in input_data['lidar_left'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp

        signals.floatSignals.append(FloatSignal("DistanceToFrontUS_left", minDistance))

        # distance_to_front_right
        minDistance = 2.0
        for point in input_data['lidar_right'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp

        signals.floatSignals.append(FloatSignal("DistanceToFrontUS_right", minDistance))

        # VelocityIn
        signals.floatSignals.append(FloatSignal("VelocityIn", 0.4 if self._previous_speed[0] > 0.4 else self._previous_speed[0]))

        # LaneDetection
        # signals.floatSignals.append(FloatSignal("LD_server_connected", 1.0))
        # signals.floatSignals.append(FloatSignal("LD_present_right", 1.0))
        # signals.floatSignals.append(FloatSignal("LD_present_left", 1.0))

        waypoint: carla.Waypoint = CarlaDataProvider.get_world().get_map().get_waypoint(
            self.ego_vehicle.get_location(), project_to_road=True)

        normal_vector_right: carla.Vector3D = waypoint.transform.get_right_vector()
        base_point_plane = waypoint.transform.location

        vector_from_car_to_plane: carla.Vector3D = self.ego_vehicle.get_location() - base_point_plane
        signed_distance = vector_from_car_to_plane.dot(normal_vector_right)
        # Negative -> left
        # Positive -> right
        distance_lane_from_middle = 2
        distance_left = distance_lane_from_middle + signed_distance
        distance_right = distance_lane_from_middle - signed_distance

        # print(f"Lane distances: left: {distance_left} right: {distance_right}")

        # Add signals for correct lane detection function
        signals.floatSignals.append(FloatSignal("LD_Distance_Right", distance_right))
        signals.floatSignals.append(FloatSignal("LD_Distance_Left", distance_left))

        signals.boolSignals.append(BoolSignal("LD_server_connected", True))
        signals.boolSignals.append(BoolSignal("LD_present_right", True))
        signals.boolSignals.append(BoolSignal("LD_present_left", True))

        resp = self._do_step_service(signals, rospy.get_rostime())

        # Search for steering and motor value in the signals
        steering = None
        for float_signal in resp.result.floatSignals:
            if float_signal.name == 'MotorValue':
                self._speed = float_signal.value
            elif float_signal.name == "SteeringValue":
                steering = float_signal.value

        if not steering:
            print("SteeringValue not found. Assuming no steering")
            steering = 6000.0

        # Converting steering to -1.0 to 1.0
        # 6000 is middle, and maxiumum deviation is 1800 to either side
        steering -= 6000.0
        steering /= 1800.0

        if self._speed <= 0.2 and (self._previous_speed[0] > self._speed or
                                   self._previous_speed[1] > self._speed or self._previous_speed[2] > self._speed):
            self._previous_speed.pop()
            self._previous_speed.insert(0, self._speed)
            return carla.VehicleControl(throttle=0.0,
                                        steer=steering,
                                        brake=1.0,
                                        hand_brake=False,
                                        reverse=False,
                                        manual_gear_shift=False,
                                        gear=1)
        elif self._previous_speed[0] > self._speed or self._previous_speed[1] > self._speed or self._previous_speed[2] > self._speed:
            self._previous_speed.pop()
            self._previous_speed.insert(0, self._speed)
            return carla.VehicleControl(throttle=0.0,
                                        steer=steering,
                                        brake=0.5,
                                        hand_brake=False,
                                        reverse=False,
                                        manual_gear_shift=False,
                                        gear=1)

        self._previous_speed.pop()
        self._previous_speed.insert(0, self._speed)

        if self._speed > 0.4:
            self._speed = 0.4

        return carla.VehicleControl(throttle=self._speed,
                                           steer=steering,
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
                    'type': 'sensor.camera.rgb',
                    'x': 0.7, 'y': 0.0, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                    'width': 800, 'height': 600, 'fov': 100,
                    'id': 'center'
                }
            )

        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': 0.0, 'z': 0.5,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
                    'channels': 1, 'range': 20.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 10.0, 'lower_fov': 0.0,
                    'horizontal_fov': 90.0, 'id': 'lidar'
                }
            )
        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': -0.7, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
                    'channels': 8, 'range': 20.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 10, 'id': 'lidar_left'
                }
            )
        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': 0.7, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
                    'channels': 8, 'range': 20.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 10, 'id': 'lidar_right'
                }
            )

        return sensors

    def destroy(self):
        if self._visual is not None:
            self._visual.quit = True
