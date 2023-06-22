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
### Utility functions

The following utility functions are being offered:
- `untoggle_environment_objects(world, semantic_tags)`: removes specified environment objects from the world
  - `world`: carla.World
  - `semantic_tags`: list(carla.CityObjectLabel). Specifies which objects are NOT being removed
- `change_color_texture_of_objects(world, filter_criteria, color, width, height, material)`: changes the color and texture of filtered objects
  - `world`: carla.World
  - `filter_criteria`: String. Only objects, whose name contains the filter_criteria, are being painted. E.g. `BP_StreetLight_6` to only paint a certain street light.
  - `color`: carla.Color
  - `width`: int
  - `height`: int
  - `material`: carla.MaterialParameter
- `spawn_props(world, prop)`: spawns 10 props on road lines in front of the ego vehicle. Only works for `LK_highway_exit.xosc` scenario
  - `world`: carla.World
  - `prop`: String, specifying which prop is being spawned e.g. `static.prop.creasedbox03`. The props catalogue can be found [here](https://carla.readthedocs.io/en/latest/catalogue_props/).

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
