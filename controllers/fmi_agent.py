# Copyright (c) 2021 Universitat Autonoma de Barcelona (UAB)
# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy
from numpy.linalg import norm

from rosco.srv import *
from rosco.msg import *
from visualizations.real import CameraView
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

    def __init__(self, simulator):
        super().__init__("")
        if not simulator.get_client().get_world().get_settings().no_rendering_mode:
            self._visual = CameraView('center')

    def setup(self, _):
        self._agent = None
        
        for actor in CarlaDataProvider.get_world().get_actors():
            if 'role_name' in actor.attributes and actor.attributes['role_name'] == 'hero':
                self._ego = actor
                break        
        
        rospy.init_node('fmi_agent')
        
        #initialize and call the masterInit service
        rospy.wait_for_service('master/init')
        init_master = rospy.ServiceProxy('master/init', MasterInitService)
        print('calling init/master service')
        init_master('/opt/workspace/src/rosco/share/config.json')
        
        # initialize the doStepsUntil Service
        rospy.wait_for_service('master/doStepsUntil')
        self._do_step_service = rospy.ServiceProxy('master/doStepsUntil', DoStepsUntilService, persistent=True)
        print('initialized do step service')

    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)
        
        signals = SignalsMessage()
              
        #distance_to_front
        minDistance = 2.0
        for point in input_data['lidar'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp
        
        signals.floatSignals.append(FloatSignal("DistanceToFrontLaser", minDistance))        
          
        #distance_to_front_left
        minDistance = 2.0
        for point in input_data['lidar_left'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp
        
        signals.floatSignals.append(FloatSignal("DistanceToFrontUS_left", minDistance))        
           
        #distance_to_front_right
        minDistance = 2.0
        for point in input_data['lidar_right'][1]:
            tmp = norm(point[:-1])/10.0
            if (tmp < minDistance):
                minDistance = tmp
        
        signals.floatSignals.append(FloatSignal("DistanceToFrontUS_right", minDistance))
        
        #VelocityIn
        signals.floatSignals.append(FloatSignal("VelocityIn", 0.4 if self._previous_speed[0] > 0.4 else self._previous_speed[0]))

        resp = self._do_step_service(signals, rospy.get_rostime())
        
        for float_signal in resp.result.floatSignals:
            if float_signal.name == 'MotorValue':
                self._speed = float_signal.value
                
        if self._speed <= 0.2 and (self._previous_speed[0] > self._speed or 
                                   self._previous_speed[1] > self._speed or self._previous_speed[2] > self._speed):
            self._previous_speed.pop()
            self._previous_speed.insert(0, self._speed)
            return carla.VehicleControl(throttle=0.0,
                                        steer=0.0,
                                        brake=1.0,
                                        hand_brake=False,
                                        reverse=False,
                                        manual_gear_shift=False,
                                        gear=1)
        elif self._previous_speed[0] > self._speed or self._previous_speed[1] > self._speed or self._previous_speed[2] > self._speed:
            self._previous_speed.pop()
            self._previous_speed.insert(0, self._speed)
            return carla.VehicleControl(throttle=0.0,
                                        steer=0.0,
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
                    'x': 1.6, 'y': -0.5, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 135.0,
                    'channels': 8, 'range': 10.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 30, 'id': 'lidar_left'
                }
            )
        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': 0.5, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 225.0,
                    'channels': 8, 'range': 10.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 30, 'id': 'lidar_right'
                }
            )
        return sensors

    def destroy(self):
        if self._visual is not None:
            self._visual.quit = True
