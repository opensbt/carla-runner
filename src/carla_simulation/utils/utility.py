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

    for semantic_tag in semantic_tags:
        env_obj = world.get_environment_objects(semantic_tag)
        env_obj_ids = [obj.id for obj in env_obj]
        world.enable_environment_objects(set(env_obj_ids), True)