# CARLA Interface for Scenario-based Testing

Evaluate scenarios using a highly parallelizable CARLA setup!

```
from carla_simulation.balancer import Balancer

b = Balancer(
    directory = '/tmp/scenarios',
    jobs = 1,
    visualization = True
)

b.start()

evaluation = b.run()

b.stop()
```

## Prerequisites

### Environment

Make sure that the following environment variables are set correctly:

* All `*_PATH` variables must point to the respective local repositories to be cloned from the following URLs:
    * `ROSCO_PATH`: https://git.fortiss.org/ff1/rosco.git
    * `OPENSBT_CORE_PATH`: https://git.fortiss.org/opensbt/opensbt-core.git
    * `OPENSBT_RUNNER_PATH`: https://git.fortiss.org/opensbt/carla-runner.git
    * `CARLA_PATH`: https://github.com/carla-simulator/carla.git
    * `SCENARIORUNNER_PATH`: https://github.com/carla-simulator/scenario_runner.git

If you are using an IDE, this can usually be done through some run configuration options.

As example when cloning the  OpenSBT Runner into `~/projects/carla-runner`, and then cloning all other repos inside the runner,
the following bash commands can be used to set up the environment for launching the software via the bash terminal.
```bash
OPENSBT_RUNNER=~/projects/carla-runner

export ROSCO_PATH=$OPENSBT_RUNNER/rosco
export OPENSBT_CORE_PATH=$OPENSBT_RUNNER/opensbt-core
export OPENSBT_RUNNER_PATH=$OPENSBT_RUNNER/carla-runner
export CARLA_PATH=$OPENSBT_RUNNER/carla
export SCENARIORUNNER_PATH=$OPENSBT_RUNNER/scenario_runner

export PYTHONPATH=$PYTHONPATH:${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:${CARLA_ROOT}/PythonAPI/carla:${SCENARIO_RUNNER_ROOT}
```
Now it is possible to start the runner via the commands `~/projects/carla-runner/ && python test.py`

### Docker

Instructions to install Docker are available [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository). The NVIDIA Container Toolkit can be installed as described [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide).

### Functional Mock-up Interface (FMI)

In case the [FMI](https://fmi-standard.org/)-based agent shall be used, the `SHARE_PATH` must point to the directory containing the Functional Mock-up Units (FMUs). As of now, it is recommended to replace the example files in ROSCo's `share/` folder with the FMUs and configuration of the system to be simulated. Finally, the launch configuraiton in ROSCo's `launch/` folder must be adapted to reflect this change.

## Getting Started

The easiest way to get the CARLA interface up and running is to build it as a Python package and install it.

To build the package, run `python -m build` in the repository's root directory. Once completed, install the `*.whl` package found in the newly created `dist/` folder via `python -m pip install /path/to/the/package.whl`.

Next, import the package:

```
from carla_simulation.balancer import Balancer
```

Now, an instance of the `Balancer` can be created:

```
b = Balancer(
    directory = '/tmp/scenarios',
    jobs = 1,
    visualization = True
)
```

Here, the directory containing the scenarios to be executed must be specified. The `jobs` variable can be used set the degree of parallelization. It is optional an set to `1` by default. The `visualization` can be set to `True` in case a 3D rendering of the scenario execution shall be shown. It is off by default, as the visualization severly impacts the framework's performance.

The balancer's infrastructure can be started as follows:

```
b.start()
```

This can take some time, especially if the necessary Docker images are not yet available and need to be built first. Next, a server and a client container will be started for each job specified in the balancer's constructor.

Now, the scenarios can be run:

```
evaluation = b.run()
```

The `run()` function will execute all scenarios in the directory specified in the balancer's constructor and return a list of evaluation metrics - a set of metrics for each scenario.

Finally, the balancer's Docker infrastructure can be stopped and removed via its `stop()` function:

```
b.stop()
```

### Functions to change the simulation environment

The following functions are being offered in `environment.py`:
- `untoggle_environment_objects(world, semantic_tags)`: removes specified environment objects from the world, see [here](https://carla.readthedocs.io/en/latest/python_api/#carlacityobjectlabel), which can be untoggled
- `change_color_texture_of_objects(world, filter_criteria, color, width, height, material)`: changes the color and the texture of filtered objects. E.g. `filter_criteria=BP_StreetLight_6` to only paint a certain street light.
- `spawn_prop(world, prop, transform)`: spawns a specified prop from the [props catalogue](https://carla.readthedocs.io/en/latest/catalogue_props/) at the location specified in the transform object
- `change_vehicle_physics(vehicle, torque_curve, max_rpm, moi, damping_rate_full_throttle, damping_rate_zero_throttle_clutch_engaged, damping_rate_zero_throttle_clutch_disengaged, clutch_strength, final_ratio, mass, drag_coefficient, center_of_mass, adjust_wheels, tire_frictions, damping_rates, radii, max_brake_torques)`: applies physics controls to the specified vehicle. Check the documentation [here](https://carla.readthedocs.io/en/latest/python_api/#carla.VehiclePhysicsControl) and [here](https://carla.readthedocs.io/en/latest/python_api/#carlawheelphysicscontrol) to find out more about the parameter.

The following effects can be achieved by adjusting the scenario file:
- `changing the weather`: see the documentation for weather [here](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/content/Weather.html).
  - `sunny weather`: `<Sun intensity="1.0" azimuth="5.2" elevation="0.35"/>`. Hardcoded for `LK_highway_exit.xosc` scenario
  - `foggy weather`: `<Fog visualRange="1.0"/>`
  - `rainy weather`: `<Precipitation precipitationType="rain" intensity="1.0"/>`
- `set the maximum speed, acceleration and deceleration of the ego vehicle`: e.g. `<Performance maxSpeed="69.444" maxAcceleration="200" maxDeceleration="10.0"/>`. see the documentation for performance [here](https://releases.asam.net/OpenSCENARIO/1.0.0/Model-Documentation/content/Performance.html)

The following effects can be achieved by adjusting the parameters for the sensors in the `fmi.py` file:
- `LIDAR`:
  - `atmosphere_attenuation_rate`: coefficient that measures the LIDAR intensity loss, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)
  - `noise_stddev`: standard deviation of the noise model of the LIDAR sensor, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)
- `camera`: the camera offers multiple parameters to change the `motion blur` and the `calibration`, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera) 

### SOTIF triggering events

| SOTIF Triggering Events | Implementation |
| ------------------------| ---------------|
| The camera sensor may not detect the lane boundaries because the lane markings are partially or fully covered | use RoadPainter to add snow or water layer on the street |
| Obstructions may block the camera's view of lane markings, vehicles, or other objects. | use the `spawn_props()` method |
| The camera may have deteriorated performance in environmental conditions that reduce visibility, such as weather or low lighting. | change the weather in the scenario file, e.g. blinding sun, fog, rain etc.|
| Environmental noise factors, such as light reflection or shadows, may affect the sensor's ability to detect lane markings, vehicles, or other objects. | <ul><li>`camera`: offers multiple parameters to change the motion blur, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera)</li><li>`LIDAR`: offers `noise_stddev` to add environmental noise, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)</li></ul> |
| Atmospheric attenuation leads to LIDAR inensity loss which may affect the LIDAR's ability to detect lane markings, vehicles, or other objects. | `LIDAR` offers `atmosphere_attenuation_rate`, which measures the LIDAR's intensity loss, see [here](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor) |
| The camera may not detect roadside landmarks, such as concrete barriers or guardrails, if there is low contrast between the landmarks and the roadway or other environmental features. | spawn the props with the `spawn_props()` function and afterwards change the texture with the `change_color_texture_of_objects()` |
| The camera may not detect lane markings if the lane markings have low contrast with the pavement or are below a minimum size or quality. | use RoadPainter to change the lane markings |
| Due to wear and tear, the vehicle may not function safely. | <ul><li>use the `change_vehicle_physics()` function to depict functional misbehaviour</li><li>limit the vehicles maximum speed, acceleration and/or deceleration in the scenario file</li><li>manipulate the `carla.VehicleControl` object in the `run_step()` function of the `fmi.py` file to change the vehicles driving behaviour</li></ul> |
| The vehicle or object in an adjacent lane may be outside the camera's field-of-view. | Adjust the scenario file in such a corresponding way. |
| If lead vehicle tracking is used in the absence of clear lane markings, the lead vehicle may exceed the visual range of the camera | Remove lane markings in RoadPainter and adjust the front vehicles behaviour in the scenario to perform multiple fast lane changes in a row. |
| The camera, radar, lidar as well as the object trail/tracker algorithm may have limitations individually tracking multiple objects that are close together and moving at similar speeds | Create scenario with at least two motorbikes or bicycles close next to each other, which look the same |
| The roadway geometry, such as curvature or grade, may prevent the camera from correctly determining the distance to other vehicles, including the lead vehicle | Add a lead vehicle to the `LK_roundabout.xosc` file or use a similar scenario |
| The camera, radar and lidar may not detect certain environmental features with sufficient confidence, such as guardrails. | Add the carla.CityObjectLabel.GuardRail to the `untoggle_environment_objects()` function in order to keep guardrails visible. |
| The LIDAR and radar may not detect vehicles with thin profiles, such as motorcycles or bicycles, or objects below a certain size | Adjust the scenario file to spawn motorcycles or bicycles in front of the vehicle |
| The lane model algorithm may incorrectly categorize other roadway features, such as off-ramps or branching lanes, as a continuation of the current travel lane. | Simulate a complex traffic situation with multiple lane markings to confuse the lane model algorithm |
| The lane model algorithm perceives other environmental features as the lane lines | Add props (e.g creased box 03) with the `spawn_props()` function in a way that the props look like lane markings. |
| In the absence of clear lane markings, the object trail/tracker algorithm may track the incorrect lead vehicle. | create scenario without lane markins and with multiple vehicles in front of the ego vehicle |
| In the absence of clear lane markings, the object trail/tracker algorithm tracks a lead vehicle that is not staying centered in the travel lane (e.g., swerving, exiting roadway, changing lanes). | create scenario without lane markings and a vehicle, perform a lane change in front of the ego vehicle | 
| The object trail/tracker algorithm may incorrectly assign the track of an object to the incorrect lane. | create scenario with multiple lanes and place a vehicle 2 lines next to the ego vehicle |
| The object trail/tracker algorithm may incorrectly determine that a vehicle in the adjacent lane is changing to another lane. | create scenario, where a vehicle is steering left and right in the lane to throw off the object trail/tracker algorithm. |
| The object trail/tracker algorithm may not detect an object moving in front of the host vehicle during a lane change. | create scenario, where the ego vehicle changes lane |
| The object trail/tracker may not correctly detect or classify the entire vehicle or object. | create scenario and spawn an [European HGV](https://carla.readthedocs.io/en/latest/catalogue_vehicles/) |
| In the absence of clear lane markings or landmarks, the road model algorithm incorrectly establishes the travel lane and/or the target lane. | test the ego vehicle in a scenario without any lane markings |
| The road model algorithm incorrectly estimates the road curvature and reports the incorrect curvature to the steerable path algorithms | see how the ego vehicle in a roundabout, e.g. in `LK_roundabout.xosc` |
| The environmental or roadway conditions may change suddenly, causing the system to reach the limits of its ODD sooner than expected. | The lane suddenly stops in front of a desert |
| The highway chauffeur system may be incapable of safely bringing the vehicle to a stop in the middle of a maneuver. | During a lane keep maneuver another vehicle cuts right in front of the ego vehicle and fully brakes. |

### Maps, which can be simulated in the docker container
- Town01
- TownXX_Opt, while XX can be any number from 01 up to 07, 10 and if the additional maps are installed 11 and 12. 

### Run without Docker

The following code changes have to be done:
- In `infrastructure.py`, change the NETWORK mode to `host`, connect the carla Client to `127.17.0.1` and remove any calls of the `get_address()` function
- Optionally to avoid overhead in `infrastructure.py`, change the server image to `ubuntu` and set the command for the server container in `create_server_container()` to `sleep infinity`
- In `runner.py`, change the host's IP address to `127.17.0.1`

Then download the CARLA simulator and execute it with `export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"  && ./CarlaUE4.sh` in order to use the NVIDIA GPU locally

### Visual Studio Code

If you use [Visual Studio Code](https://code.visualstudio.com/), the following launch file might be useful reference:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Start",
            "type": "python",
            "request": "launch",
            "program": "test.py",
            "console": "integratedTerminal",
            "justMyCode": false,
            "env": {
                "ROSCO_PATH": "/opt/ROSCo",
                "SHARE_PATH": "/opt/ROSCo/share",
                "CARLA_PATH": "/opt/CARLA/Simulator",
                "SCENARIORUNNER_PATH": "/opt/CARLA/ScenarioRunner",
                "OPENSBT_CORE_PATH": "/opt/OpenSBT/Core",
                "OPENSBT_RUNNER_PATH": "/opt/OpenSBT/Runner",
            }
        }
    ]
}
```
