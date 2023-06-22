# Copyright (c) 2022 fortiss GmbH
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time
import carla

from pathlib import Path

from srunner.scenarios.open_scenario import OpenScenario
from srunner.scenarioconfigs.openscenario_configuration import OpenScenarioConfiguration
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenario_manager import ScenarioManager

from carla_simulation.utils.utility import untoggle_environment_objects
from carla_simulation.utils.utility import spawn_props
from carla_simulation.utils.utility import change_color_texture_of_objects
from carla_simulation.utils.utility import change_weather

class Scenario:

    _xosc = None

    def __init__(self, xosc):
        self._xosc = xosc

    def simulate(self, simulator, agent, recorder):
        client = simulator.get_client()

        world = client.get_world()

        CarlaDataProvider.set_client(client)
        CarlaDataProvider.set_world(world)

        untoggle_environment_objects(world, [carla.CityObjectLabel.Roads, carla.CityObjectLabel.RoadLines])

        spawn_props(world, 'static.prop.creasedbox03')

        change_color_texture_of_objects(world, filter_criteria='', color=carla.Color(r=255, g=255, b=255, a=255), width=100, height=100, material=carla.MaterialParameter.Diffuse)

        change_weather(world)

        actor_list = CarlaDataProvider.get_world().get_actors()
        if actor_list:
            print(f"Destroying {len(actor_list)} actors")
            timout = time.time() + 30
            for actor in actor_list:
                if time.time() > timout:
                    print(f"[Scenario] Aborted clearing actors as it took longer than 30s")
                    break
                actor.destroy()

        config = OpenScenarioConfiguration(
            self._xosc,
            client,
            {}
        )

        CarlaDataProvider.set_traffic_manager_port(
            simulator.get_traffic_manager_port()
        )

        vehicles = []
        for vehicle in config.ego_vehicles:
            vehicles.append(
                CarlaDataProvider.request_new_actor(
                    vehicle.model,
                    vehicle.transform,
                    vehicle.rolename,
                    color=vehicle.color,
                    actor_category=vehicle.category
                )
            )

        # We assume there is only one ego actor, as only one agent is created.
        controller = agent(simulator, vehicles[0])

        scenario = OpenScenario(
            world,
            vehicles,
            config,
            self._xosc
        )

        recording = recorder.add_recording(
            Path(self._xosc.path).stem
        )

        manager = ScenarioManager(
            timeout = 60.0
        )
        manager.load_scenario(scenario, controller)
        client.start_recorder(
            recording,
            True
        )
        manager.run_scenario()
        client.stop_recorder()
