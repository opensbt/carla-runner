# Copyright (c) 2020 Universitat Autonoma de Barcelona (UAB)
# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

from srunner.metrics.tools.metrics_log import MetricsLog

class RawData:

    def evaluate(self, simulator, recording):
        info = simulator.get_client().show_recorder_file_info(recording, True)
        log = MetricsLog(info)

        ego_id = log.get_actor_ids_with_role_name("hero")[0]

        frames_time_list = []
        ego_location_profile = []
        ego_speed_profile = []

        start, end = log.get_actor_alive_frames(ego_id)

        collisions = log.get_actor_collisions(ego_id)

        simTime = log.get_elapsed_time(log.get_total_frame_count()-1)

        for i in range(start, end):
            frames_time_list.append(log.get_elapsed_time(i))

            ego_location = log.get_actor_transform(ego_id, i).location
            ego_location_profile.append((ego_location.x, ego_location.y))

            ego_speed = log.get_actor_velocity(ego_id,i)
            ego_speed_profile.append(ego_speed.length())

        result = {
            "simulationTime" : 0,
            "times": [],
            "location": {
                "ego" : []
            },
            "velocity": {
                "ego" : []
            },
            "collisions": [],
            "actors" : {},
            "otherParameters" : {}
        }

        result["simTime"] = simTime
        result["times"] = frames_time_list
        result["location"]["ego"] = ego_location_profile
        result["velocity"]["ego"] = ego_speed_profile
        result["collisions"] = collisions
        result["actors"] = {
            ego_id: "ego"
        }

        return result
