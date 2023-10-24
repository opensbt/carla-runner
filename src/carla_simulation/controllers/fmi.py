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

from carla_simulation.utils.manual_control import parse_keyboard_events_to_xbox
from carla_simulation.utils.sensing import process_lidar_data
from carla_simulation.utils.sensing import process_location_data

FAULT_DIR = "/tmp/faults"

class FMIAgent(AutonomousAgent):

    SENSORS_MAX_DISTANCE = 20.0
    SENSORS_OFFSET = 70.0

    VEHICLE_MAX_SPEED = 0.4
    VEHICLE_SCALE = 10
    VEHICLE_OFFSET_STEERING = 6000.0
    VEHICLE_MAX_STEERING = 1800.0

    ROAD_SCALE = 1.25

    CONTROLLER_STICK_RESOLUTION = 32768.0
    CONTROLLER_SHOULDER_RESOLUTION = 1024.0

    _agent = None
    _visual = None
    _do_step_service = None
    _activate_faultinjector_service = None
    _deactivate_faultinjector_service = None
    _previous_speed = [VEHICLE_MAX_SPEED, VEHICLE_MAX_SPEED, VEHICLE_MAX_SPEED]
    _ego_vehicle = None
    _enable_manual_control = False
    _fault = None

    def __init__(self, simulator, ego_vehicle: carla.Vehicle):
        super().__init__("")
        self._ego_vehicle = ego_vehicle
        self._enable_manual_control = simulator.is_manual_control_enabled()
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

        if self._enable_manual_control:
            print("The manual keyboard control was enabled.")

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

        if self._enable_manual_control:
            xbox_mapping = parse_keyboard_events_to_xbox()
            signals.floatSignals.append(FloatSignal(
                "LeftStick_X",
                xbox_mapping["left_stick_X"] * self.CONTROLLER_STICK_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "LeftStick_Y",
                xbox_mapping["left_stick_Y"] * self.CONTROLLER_STICK_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "RightStick_X",
                xbox_mapping["right_stick_X"] * self.CONTROLLER_STICK_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "RightStick_Y",
                xbox_mapping["right_stick_Y"] * self.CONTROLLER_STICK_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonX",
                float(xbox_mapping["button_X"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonY",
                float(xbox_mapping["button_Y"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonA",
                float(xbox_mapping["button_A"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonB",
                float(xbox_mapping["button_B"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonR1",
                float(xbox_mapping["button_R1"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonL1",
                float(xbox_mapping["button_L1"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonR2",
                float(xbox_mapping["button_R2"]) * self.CONTROLLER_SHOULDER_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonL2",
                float(xbox_mapping["button_L2"]) * self.CONTROLLER_SHOULDER_RESOLUTION
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonHome",
                float(xbox_mapping["button_home"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonStart",
                float(xbox_mapping["button_start"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonSelect",
                float(xbox_mapping["button_select"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadLeft",
                float(xbox_mapping["dpad_left"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadRight",
                float(xbox_mapping["dpad_right"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadUp",
                float(xbox_mapping["dpad_up"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadDown",
                float(xbox_mapping["dpad_down"])
            ))

        

        # Laser distance
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontLaser",
            process_lidar_data(
                input_data,
                'lidar',
                self.SENSORS_MAX_DISTANCE
            ) * 100 / self.VEHICLE_SCALE + self.SENSORS_OFFSET
        ))

        # Ultrasound distances
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_left",
            process_lidar_data(
                input_data,
                'lidar_left',
                self.SENSORS_MAX_DISTANCE
            ) * 1000 / self.VEHICLE_SCALE + self.SENSORS_OFFSET
        ))
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_right",
            process_lidar_data(
                input_data,
                'lidar_right',
                self.SENSORS_MAX_DISTANCE
            ) * 1000 / self.VEHICLE_SCALE + self.SENSORS_OFFSET
        ))

        # Velocity feedback
        signals.floatSignals.append(FloatSignal(
            "VelocityIn",
            self.VEHICLE_MAX_SPEED \
                if self._previous_speed[0] > self.VEHICLE_MAX_SPEED \
                else self._previous_speed[0]
        ))

        # Lane detection
        distance_right, distance_left = process_location_data(self._ego_vehicle)
        signals.floatSignals.append(FloatSignal(
            "LD_Distance_Right",
            distance_right * self.ROAD_SCALE))
        signals.floatSignals.append(FloatSignal(
            "LD_Distance_Left",
            distance_left * self.ROAD_SCALE))
        signals.floatSignals.append(FloatSignal(
            "LD_server_connected",
            float(True)))
        signals.floatSignals.append(FloatSignal(
            "LD_present_right",
            float(True)))
        signals.floatSignals.append(FloatSignal(
            "LD_present_left",
            float(True)))

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

        if steering_value is None:
            print("SteeringValue not found. Assuming no steering.")
            steering_value = self.VEHICLE_MIN_STEERING

        if motor_value is None:
            print("MotorValue not found. Assuming no speed.")
            motor_value = 0.0

        steering = (steering_value - self.VEHICLE_OFFSET_STEERING)/self.VEHICLE_MAX_STEERING

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

        if throttle > self.VEHICLE_MAX_SPEED:
            throttle = self.VEHICLE_MAX_SPEED

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
                    'horizontal_fov': 90.0, 'atmosphere_attenuation_rate': 0.004,
                    'noise_stddev': 0.0, 'id': 'lidar'
                }
            )
        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': -0.7, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
                    'channels': 8, 'range': 20.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 10, 'atmosphere_attenuation_rate': 0.004,
                    'noise_stddev': 0.0, 'id': 'lidar_left'
                }
            )
        sensors.append(
                {
                    'type': 'sensor.lidar.ray_cast',
                    'x': 1.6, 'y': 0.7, 'z': 1.60,
                    'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
                    'channels': 8, 'range': 20.0, 'points_per_second': 100,
                    'rotation_frequency': 0.0, 'upper_fov': 5.0, 'lower_fov': -5.0,
                    'horizontal_fov': 10, 'atmosphere_attenuation_rate': 0.004,
                    'noise_stddev': 0.0, 'id': 'lidar_right'
                }
            )

        return sensors

    def destroy(self):
        if self._visual is not None:
            self._visual.quit = True
