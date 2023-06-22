# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

def untoggle_environment_objects(world, semantic_tags):
    # toggle all environment objects off
    env_objs = world.get_environment_objects(carla.CityObjectLabel.Any)
    env_objs_ids = [obj.id for obj in env_objs]
    world.enable_environment_objects(set(env_objs_ids), False)

    #toggle the specified environment objects back on
    for semantic_tag in semantic_tags:
        env_obj = world.get_environment_objects(semantic_tag)
        env_obj_ids = [obj.id for obj in env_obj]
        world.enable_environment_objects(set(env_obj_ids), True)

def change_color_texture_of_objects(world, filter_criteria, color, width, height, material):
    objs = world.get_names_of_all_objects()
    filtered_objs = list(filter(lambda k: filter_criteria in k, objs))

    world.apply_color_texture_to_objects(filtered_objs, material, 
                                    get_texture(color, width, height))

def get_texture(color, width, height):
        texture = carla.TextureColor(width, height)
        for x in range(0, width):
            for y in range(0, height):
                texture.set(x, y, color)
        return texture

def spawn_props(world, prop):
    bp = world.get_blueprint_library().find(prop)

    world.spawn_actor(bp, carla.Transform(carla.Location(-25.5, 120.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-28.0, 124.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-31.0, 128.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-34.5, 131.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-38.5, 134.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-20.5, 120.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-23.0, 124.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-26.5, 128.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-30.0, 132.0, 0.0 ), carla.Rotation(0, 0, 0)))
    world.spawn_actor(bp, carla.Transform(carla.Location(-34.5, 136.0, 0.0 ), carla.Rotation(0, 0, 0)))

def change_vehicle_physics(vehicle):
    physics_control = vehicle.get_physics_control()

    # change the front left wheel's physical control
    front_left_wheel  = physics_control.wheels[0]
    front_left_wheel.radius = 37.0
    front_left_wheel.tire_friction = 3.5

    # change the front right wheel's physical control
    front_right_wheel = physics_control.wheels[1]
    front_right_wheel.radius = 37.0
    front_right_wheel.tire_friction = 3.5

    # change the rear left wheel's physical control
    rear_left_wheel   = physics_control.wheels[2]
    rear_left_wheel.radius = 37.0
    rear_left_wheel.tire_friction = 3.5

    # change the rear right wheel's physical control
    rear_right_wheel  = physics_control.wheels[3]
    rear_right_wheel.radius = 37.0
    rear_right_wheel.tire_friction = 3.5

    wheels = [front_left_wheel, front_right_wheel, rear_left_wheel, rear_right_wheel]
    physics_control.wheels = wheels

    vehicle.apply_physics_control(physics_control)