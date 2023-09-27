# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla

from numpy.linalg import norm
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider


def process_lidar_data(input_data, sensor_id, min_distance):
    minimum = min_distance
    for point in input_data[sensor_id][1]:
            tmp = norm(point[:-1])
            if (tmp < minimum):
                minimum = tmp
    return minimum

def process_location_data(ego_vehicle):
     # Lane detection
    waypoint: carla.Waypoint = CarlaDataProvider.get_world().get_map().get_waypoint(
        ego_vehicle.get_location(), project_to_road=True)

    # Create a normal vector going to the right from the view of the waypoint,
    # creating a plane which splits the space into left and right from the view
    # of the waypoint
    normal_vector_right: carla.Vector3D = waypoint.transform.get_right_vector()
    base_point_plane = waypoint.transform.location

    # We then calculate the signed distance of the car to the plane, to know if
    # we are left or right of the plane
    vector_from_car_to_plane: carla.Vector3D = ego_vehicle.get_location() - base_point_plane
    signed_distance = vector_from_car_to_plane.dot(normal_vector_right)

    # Negative -> Left
    # Positive -> Right
    distance_lane_from_middle = waypoint.lane_width / 2.0
    distance_left = distance_lane_from_middle + signed_distance
    distance_right = distance_lane_from_middle - signed_distance

    return distance_right, distance_left
