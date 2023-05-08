# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy
import os
import yaml
import os
import yaml
from numpy.linalg import norm

from rosco.srv import *
from rosco.msg import *
from carla_simulation.visualizations.real import CameraView
from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime
from yaml.loader import SafeLoader 


class FMIAgent(AutonomousAgent):

    _agent = None
    _route_assigned = False
    _visual = None
    _do_step_service = None
    _activate_faultinjector_service = None
    _deactivate_faultinjector_service = None
    _speed = None
    _previous_speed = [0.5, 0.5, 0.5]
    _ego = None
    _fault = None

    def __init__(self, simulator):
        super().__init__("")
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
                
        # Initialize ActivateFaultinjector Service
        rospy.wait_for_service('master/activateFaultInjector')
        self._activate_faultinjector_service = rospy.ServiceProxy('master/activateFaultInjector', ActivateFaultInjector, persistent=True)
        print('initialized activate faultinjector service')
        
        # Initialize DeactivateFaultinjector Service
        rospy.wait_for_service('master/deactivateFaultInjector')
        self._deactivate_faultinjector_service = rospy.ServiceProxy('master/deactivateFaultInjector', DeactivateFaultInjector, persistent=True)
        print('initialized deactivate faultinjector service')
   
        # Initialize ActivateFaultinjector Service
        rospy.wait_for_service('master/activateFaultInjector')
        self._activate_faultinjector_service = rospy.ServiceProxy('master/activateFaultInjector', ActivateFaultInjector, persistent=True)
        print('initialized activate faultinjector service')
        
        # Initialize DeactivateFaultinjector Service
        rospy.wait_for_service('master/deactivateFaultInjector')
        self._deactivate_faultinjector_service = rospy.ServiceProxy('master/deactivateFaultInjector', DeactivateFaultInjector, persistent=True)
        print('initialized deactivate faultinjector service')
   
    def setFault(fault):
        FMIAgent._fault = fault
        
    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)
        
        signals = SignalsMessage()
        print(self._fault)
        timestamp = GameTime.get_time()
        
        if timestamp > 3.0 and timestamp < 3.1:
            with open('/opt/workspace/share/faults/omission_1.yaml') as f:
                data = yaml.safe_load(f)
                print(data['faultInjection'])
                dict = data['faultInjection']
            
            faultInjectionStructure = FaultInjectionStructure()
            #faultInjectionStructure = data['faultInjection']       
            faultInjectionStructure.faultModel = dict.get("faultModel")
            faultInjectionStructure.signalNames = dict.get('signalNames')
            faultInjectionStructure.parameters = dict.get('parameters').encode().decode('unicode_escape')
            injected = self._activate_faultinjector_service(faultInjectionStructure)
            print("activated faultinjection")
            
        if timestamp > 9.0 and timestamp < 9.1:  
            injected = self._deactivate_faultinjector_service()
            print(injected.error)
            print("deactivated faultinjection")  
        
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
