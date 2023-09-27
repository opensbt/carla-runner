# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import rospy

from rosco.srv import *
from rosco.msg import *
from carla_simulation.visualizations.real import CameraView
from srunner.autoagents.autonomous_agent import AutonomousAgent
from carla_simulation.controllers.manual_control import HumanAgent

from carla_simulation.utils.sensing import process_lidar_data
from carla_simulation.utils.sensing import process_location_data

class FMIAgent(AutonomousAgent):

    SENSOR_MAX_DISTANCE = 20.0
    VEHICLE_SCALE = 10
    ROAD_SCALE = 1.25

    _agent = None
    _visual = None
    _do_step_service = None
    _previous_speed = [0.5, 0.5, 0.5]
    _ego_vehicle = None
    _manual_controller = None
    _enable_manual_control = False

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

        # Initialize the agent for the manual controller
        self._manual_controller = HumanAgent("")
        #print("\n initialized\n")
        self._manual_controller.setup("")

        # Initialize the doStepsUntil service.
        rospy.wait_for_service('master/doStepsUntil')
        self._do_step_service = rospy.ServiceProxy('master/doStepsUntil', DoStepsUntilService, persistent=True)
        print('initialized do step service')

    def run_step(self, input_data, _):
        if self._visual is not None:
            self._visual.run(input_data)

        keyboard_data = self._manual_controller.run_step(None, 0)

        signals_in = self.sense(input_data, keyboard_data)

        signals_out = self._do_step_service(
            signals_in,
            rospy.get_rostime()
        )

        control = self.act(signals_out)

        return control

    def booleanToBinary(self, booleanValue):
        if booleanValue:
            return 1.0
        else:
            return 0.0

    def sense(self, input_data, xbox_mapping):
        signals = SignalsMessage()

        if self._enable_manual_control:
            # Controller input
            signals.floatSignals.append(FloatSignal(
                "LeftStick_X",
                xbox_mapping["left_stick_X"] * 32768.0
            ))
            signals.floatSignals.append(FloatSignal(
                "LeftStick_Y",
                xbox_mapping["left_stick_Y"] * 32768.0
            ))
            signals.floatSignals.append(FloatSignal(
                "RightStick_X",
                xbox_mapping["right_stick_X"] * 32768.0
            ))
            signals.floatSignals.append(FloatSignal(
                "RightStick_Y",
                xbox_mapping["right_stick_Y"] * 32768.0
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonX",
                self.booleanToBinary(xbox_mapping["button_X"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonY",
                self.booleanToBinary(xbox_mapping["button_Y"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonA",
                self.booleanToBinary(xbox_mapping["button_A"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonB",
                self.booleanToBinary(xbox_mapping["button_B"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonR1",
                self.booleanToBinary(xbox_mapping["button_R1"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonL1",
                self.booleanToBinary(xbox_mapping["button_L1"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonR2",
                self.booleanToBinary(xbox_mapping["button_R2"]) * 1024.0
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonL2",
                self.booleanToBinary(xbox_mapping["button_L2"]) * 1024.0
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonHome",
                self.booleanToBinary(xbox_mapping["button_home"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonStart",
                self.booleanToBinary(xbox_mapping["button_start"])
            ))
            signals.floatSignals.append(FloatSignal(
                "ButtonSelect",
                self.booleanToBinary(xbox_mapping["button_select"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadLeft",
                self.booleanToBinary(xbox_mapping["dpad_left"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadRight",
                self.booleanToBinary(xbox_mapping["dpad_right"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadUp",
                self.booleanToBinary(xbox_mapping["dpad_up"])
            ))
            signals.floatSignals.append(FloatSignal(
                "DPadDown",
                self.booleanToBinary(xbox_mapping["dpad_down"])
            ))

        # Laser distance
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontLaser",
            process_lidar_data(input_data, 'lidar', self.SENSOR_MAX_DISTANCE) * 100 / self.VEHICLE_SCALE + 70
        ))

        # Ultrasound distances
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_left",
            process_lidar_data(input_data, 'lidar_left', self.SENSOR_MAX_DISTANCE) * 1000 / self.VEHICLE_SCALE + 70
        ))
        signals.floatSignals.append(FloatSignal(
            "DistanceToFrontUS_right",
            process_lidar_data(input_data, 'lidar_right', self.SENSOR_MAX_DISTANCE) * 1000 / self.VEHICLE_SCALE + 70
        ))

        # Velocity feedback
        signals.floatSignals.append(FloatSignal(
            "VelocityIn",
            0.4 if self._previous_speed[0] > 0.4 else self._previous_speed[0]
        ))

        # Lane detection
        distance_right, distance_left = process_location_data(self._ego_vehicle)
        signals.floatSignals.append(FloatSignal("LD_Distance_Right", distance_right * self.ROAD_SCALE))
        signals.floatSignals.append(FloatSignal("LD_Distance_Left", distance_left * self.ROAD_SCALE))
        signals.floatSignals.append(FloatSignal("LD_server_connected", 1.0))
        signals.floatSignals.append(FloatSignal("LD_present_right", 1.0))
        signals.floatSignals.append(FloatSignal("LD_present_left", 1.0))

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
            steering_value = 6000.0

        if motor_value is None:
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
