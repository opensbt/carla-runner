# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

def show(world, labels):
    """
    Untoggle the environment objects from the world, which are NOT specified in the labels.

    Args:
        world (carla.World)
        labels (list(carla.CityObjectLabel)): Specifies the city object labels, which are NOT being toggled off.

    Example:
        show(world, [
            carla.CityObjectLabel.Roads,
            carla.CityObjectLabel.RoadLines])
    """

    # Toggle all environment objects off.
    env_objs = world.get_environment_objects(carla.CityObjectLabel.Any)
    env_objs_ids = [obj.id for obj in env_objs]
    world.enable_environment_objects(set(env_objs_ids), False)

    # Toggle the specified environment objects back on.
    for label in labels:
        env_obj = world.get_environment_objects(label)
        env_obj_ids = [obj.id for obj in env_obj]
        world.enable_environment_objects(set(env_obj_ids), True)

def color(world, filter_criteria, color, width, height, material):
    """
    Change the color and texture of specified objects in the simulation.

    Args:
        world (carla.World)
        filter_criteria (String): Only paint objects, which contain filter_criteria in their name.
        color (carla.Color): Color, which the objects are being painted.
        width (int): Specifies the width of the painted texture.
        height (int): Specifies the height of the painted texture.
        material (carla.MaterialParameter): Paints all places of the object, which have the specified material parameter.

    Example:
        color(world,
            filter_criteria='BP_StreetLight_6',
            color=carla.Color(r=255, g=255, b=255, a=255),
            width=100, height=100,
            material=carla.MaterialParameter.Diffuse)
    """

    objs = world.get_names_of_all_objects()
    filtered_objs = list(filter(lambda k: filter_criteria in k, objs))

    world.apply_color_texture_to_objects(
        filtered_objs,
        material,
        _get_texture(color, width, height))

def spawn(world, prop, transform):
    """
    Spawns a prop in a specified location with a specified rotation.

    Args:
        world (carla.World)
        prop (String): Specifies the prop.
        transform (carla.Transform): Specifies the location and rotation of the to be spawned prop.

    Example:
        spawn(world,
            'static.prop.creasedbox03',
            carla.Transform(
                carla.Location(-25.5, 120.0, 0.0 ),
                carla.Rotation(0, 0, 0)))
    """

    bp = world.get_blueprint_library().find(prop)
    world.spawn_actor(bp, transform)

def set_vehicle_physics(vehicle, torque_curve=None, max_rpm=None, moi=None, damping_rate_full_throttle=None, damping_rate_zero_throttle_clutch_engaged=None,
                           damping_rate_zero_throttle_clutch_disengaged=None, clutch_strength=None, final_ratio=None, mass=None, drag_coefficient=None,
                           center_of_mass=None, adjust_wheels=[0, 0, 0, 0], tire_frictions=[None, None, None, None], damping_rates=[None, None, None, None],
                           radii=[None, None, None, None], max_brake_torques=[None, None, None, None]):
    """
    Set the physics of the vehicle by adjusting the wheels and, e.g., changing the center of mass.
    Check the following pages for documentation on the parameter:
        - https://carla.readthedocs.io/en/latest/python_api/#carlavehiclephysicscontrol
        - https://carla.readthedocs.io/en/latest/python_api/#carla.WheelPhysicsControl

    Args:
        vehicle (carla.Vehicle): Specifies the vehicle for the physics change.
        torque_curve (list(carla.Vector2D))
        max_rpm (float)
        moi (float)
        damping_rate_full_throttle (float)
        damping_rate_zero_throttle_clutch_engaged (float)
        damping_rate_zero_throttle_clutch_disengaged (float)
        clutch_strength (float)
        final_ratio (float)
        mass (float)
        drag_coefficient (flaot)
        center_of_mass (carla.Vector3D)
        adjust_wheels (list(int)): List, which specifies which wheel should be modified (1) or not (0). This list should have 4 elements,
        where index 0 corresponds to the front left wheel, index 1 corresponds to the front right wheel, index 2 corresponds to the back
        left wheel and index 3 corresponds to the back right wheel. For 2 wheeled vehicles, set the same values for both front and back wheels.
        tire_frictions (list(float)): Represents the tire_friction of each wheels.
        damping_rates (list(float)): Represents the damping_rate of each wheels.
        radii (list(float)): represents The radius of each wheels.
        max_brake_torques (list(float)): Represents the max_brake_torque of each wheel.

    Example:
        set_vehicle_physics(vehicle=self._ego_vehicle, adjust_wheels=[1, 1, 0, 0], radii=[30.0, 100.0, 60.0, 40.0])

        set_vehicle_physics(vehicle=self._ego_vehicle, torque_curve=[carla.Vector2D(x=0, y=400), carla.Vector2D(x=1300, y=600)],
                               max_rpm=10000, moi=1.0, damping_rate_full_throttle=0.0, damping_rate_zero_throttle_clutch_engaged=0.0,
                               damping_rate_zero_throttle_clutch_disengaged=0.0, clutch_strength=10, final_ratio=1.0, mass=10000, drag_coefficient=0.25,
                               adjust_wheels=[1, 1, 1, 1], tire_frictions=[10.0, 15.0, 5.0, 0.0], damping_rates=[10.0, 10.0, 1.0, 1.0], radii=[37.0, 37.0, 37.0, 37.0],
                               max_brake_torques=[5.0, 10.0, 5.0, 10.0])
    """

    physics_control = vehicle.get_physics_control()

    if torque_curve is not None:
        physics_control.torque_curve = torque_curve

    if max_rpm is not None:
        physics_control.max_rpm = max_rpm

    if moi is not None:
        physics_control.moi = moi

    if damping_rate_full_throttle is not None:
        physics_control.damping_rate_full_throttle = damping_rate_full_throttle

    if damping_rate_zero_throttle_clutch_engaged is not None:
        physics_control.damping_rate_zero_throttle_clutch_engaged = damping_rate_zero_throttle_clutch_engaged

    if damping_rate_zero_throttle_clutch_disengaged is not None:
        physics_control.damping_rate_zero_throttle_clutch_disengaged = damping_rate_zero_throttle_clutch_disengaged

    if clutch_strength is not None:
        physics_control.clutch_strength = clutch_strength

    if final_ratio is not None:
        physics_control.final_ratio = final_ratio

    if mass is not None:
        physics_control.mass = mass

    if drag_coefficient is not None:
        physics_control.drag_coefficient = drag_coefficient

    if center_of_mass is not None:
        physics_control.center_of_mass = center_of_mass

    wheels = []
    for index, adjust_wheel, tire_friction, damping_rate, radius, max_brake_torque in zip(range(4), adjust_wheels, tire_frictions, damping_rates, radii, max_brake_torques):
        wheel = physics_control.wheels[index]
        if adjust_wheel:
            if tire_friction is not None:
                wheel.tire_friction = tire_friction

            if damping_rate is not None:
                wheel.damping_rate = damping_rate

            if radius is not None:
                wheel.radius = radius

            if max_brake_torque is not None:
                wheel.max_brake_torque = max_brake_torque

        wheels.append(wheel)

    physics_control.wheels = wheels

    vehicle.apply_physics_control(physics_control)

def _get_texture(color, width, height):
    """
    Creates the texture for the color() function.

    Args:
        color (carla.Color): Color, which the objects are being painted.
        width (int): Specifies the width of the painted texture.
        height (int): Specifies the height of the painted texture.
    """

    texture = carla.TextureColor(width, height)
    for x in range(0, width):
        for y in range(0, height):
            texture.set(x, y, color)
    return texture
