# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import carla
import xml.etree.ElementTree as et


def prepare(client, xosc):
    # Get map required for XOSC.
    scenario_map = et.parse(xosc.path).find('RoadNetwork/LogicFile').get('filepath')

    # Get map currently loaded in CARLA.
    server_map = client.get_world().get_map().name.split('/')[-1]

    # Check if the correct map is already loaded.
    if server_map != scenario_map:
        print(f" Loading map. ", end='')
        client.load_world(scenario_map)
    else:
        # The correct map is already present and is not reloaded.
        print(f" Map loaded. ", end='')
