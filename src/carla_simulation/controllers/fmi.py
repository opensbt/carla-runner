# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy
import yaml

from rosco.srv import *
from rosco.msg import *
from carla_simulation.visualizations.real import CameraView
from srunner.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.timer import GameTime

from carla_simulation.utils.sensing import process_lidar_data
from carla_simulation.utils.sensing import process_location_data

FAULT_DIR = "/tmp/faults"

class FMIAgent(AutonomousAgent):

    SENSOR_MIN_DISTANCE = 2.0

    _agent = None
    _visual = None
    _do_step_service = None
    _activate_faultinjector_service = None
    _deactivate_faultinjector_service = None
    _previous_speed = [0.5, 0.5, 0.5]
    _ego_vehicle = None
    _fault = None

    def __init__(self, simulator, ego_vehicle: carla.Vehicle):
        super().__init__("")
        self._ego_vehicle = ego_vehicle
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

        self._deactivate_faultinjector_service()
        print ("deacivate fault if fault from prior run is still activated")

    def setFault(fault):
        print("from fmi, fault set: " + fault)
        with open(fault) as f:
                data = yaml.safe_load(f)
                FMIAgent._fault = data['faultInjection']


    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)
        signals_in = self.sense(input_data)

        signals_out = self._do_step_service(
            signals_in,
            rospy.get_rostime()           
        )
                            
        control = self.act(signals_out)

        return control

    def sense(self, input_data):
        signals = SignalsMessage()
        timestamp = GameTime.get_time()
        starttime = float(self._fault['starttime'])
   

        if timestamp > starttime and timestamp < (starttime+0.1):

            dict = self._fault
            faultInjectionStructure = FaultInjectionStructure()
           
            faultInjectionStructure.faultModel = dict.get("faultModel")
            faultInjectionStructure.signalNames = dict.get('signalNames')
            faultInjectionStructure.parameters = dict.get('parameters').encode().decode('unicode_escape')
            self._activate_faultinjector_service(faultInjectionStructure)
            print("fault injected")

        # Laser distance
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontLaser",
            process_lidar_data(input_data, 'lidar', self.SENSOR_MIN_DISTANCE)
        ))

        # Ultrasound distances
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_left",
            process_lidar_data(input_data, 'lidar_left', self.SENSOR_MIN_DISTANCE)
        ))
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_right",
            process_lidar_data(input_data, 'lidar_right', self.SENSOR_MIN_DISTANCE)
        ))

        # Velocity feedback
        signals.floatSignals.append(FloatSignal(
            "VelocityIn",
            0.4 if self._previous_speed[0] > 0.4 else self._previous_speed[0]
        ))

        # Lane detection
        distance_right, distance_left = process_location_data(self._ego_vehicle)
        signals.floatSignals.append(FloatSignal("LD_Distance_Right", distance_right))
        signals.floatSignals.append(FloatSignal("LD_Distance_Left", distance_left))
        signals.boolSignals.append(BoolSignal("LD_server_connected", True))
        signals.boolSignals.append(BoolSignal("LD_present_right", True))
        signals.boolSignals.append(BoolSignal("LD_present_left", True))

        return signals

    def act(self, resp):
        # Search for steering and motor value in the signals
        steering_value = None
        motor_value = None
        for float_signal in resp.result.floatSignals:
            if float_signal.name == 'MotorValue':
                motor_value = float_signal.value
            elif float_signal.name == "SteeringValue":
                steering_value = float_signal.value

        if not steering_value:
            print("SteeringValue not found. Assuming no steering.")
            steering = 6000.0

        if not motor_value:
            print("MotorValue not found. Assuming no speed.")
            motor_value = 0.0

        # Converting steering to -1.0 to 1.0
        # 6000 is middle, and maxiumum deviation is 1800 to either side
        steering = (steering_value - 6000.0)/1800.0

        throttle = 0.0
        brake = 0.0

        # Check if our speed reduced from any of the previous speeds
        if any(speed > motor_value for speed in self._previous_speed):
            if motor_value <= 0.2:
                # Brake very hard because we want to be very slow
                throttle = 0.0
                brake = 1.0
            else:
                # Brake a little
                throttle = 0.0
                brake = 0.5
        else:
            # Otherwise just set the throttle to the commanded speed
            throttle = motor_value
            brake = 0.0

        self._previous_speed.pop()
        self._previous_speed.insert(0, motor_value)

        if throttle > 0.4:
            throttle = 0.4

        return carla.VehicleControl(throttle=throttle,
            steer=steering,
            brake=brake,
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False,
            gear=1
        )

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
