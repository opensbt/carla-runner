# Copyright (c) 2021 Universitat Autonoma de Barcelona (UAB)
# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy

from rosco.srv import *
from visualizations.real import CameraView
from srunner.autoagents.autonomous_agent import AutonomousAgent


class FMIAgent(AutonomousAgent):

    _agent = None
    _route_assigned = False
    _visual = None
    _do_step_service = None

    def __init__(self, simulator):
        super().__init__("")
        if not simulator.get_client().get_world().get_settings().no_rendering_mode:
            self._visual = CameraView('center')

    def setup(self, _):
        self._agent = None
        
        print('----   Setup function    ----')
        
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
                
        #TODO manipulate all x values into one single distance value or publish each one (but this does too many service calls)
        if(len(input_data['lidar'][1]) > 0):
            print('calling do step service with value: %f', input_data['lidar'][1][0][0])
            resp = self._do_step_service(input_data['lidar'][1][0][0], rospy.get_rostime())
        else:
            resp = self._do_step_service(10.0, rospy.get_rostime())
            print('calling do step service with value: %f', 10.0)


        motorValue = resp.motorValue
        print("Response of do step service is: %f", motorValue)
        
        #TODO manipulate the motorValue into a reasonable VehicleControl object
        if motorValue <= 0.3:
            return carla.VehicleControl(throttle=0.0,
                                        steer=0.0,
                                        brake=1.0,
                                        hand_brake=False,
                                        reverse=False,
                                        manual_gear_shift=False,
                                        gear=1)
        
        return carla.VehicleControl(throttle=motorValue,
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
                    'channels': 8, 'range': 10.0, 'points_per_second': 10000,
                    'rotation_frequency': 0.0, 'upper_fov': 10.0, 'lower_fov': 0.0,
                    'horizontal_fov': 90.0, 'id': 'lidar'
                }
            )
        return sensors

    def destroy(self):
        if self._visual is not None:
            self._visual.quit = True
