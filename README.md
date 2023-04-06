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
* In case the [FMI](https://fmi-standard.org/)-based agent shall be used, the `SHARE_PATH` must point to the directory containing the Functional Mock-up Units (FMUs).

### Docker

Instructions to install Docker are available [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository). The NVIDIA Container Toolkit can be installed as described [here](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installation-guide).

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
                "SHARE_PATH": "/tmp/FMUs",
                "CARLA_PATH": "/opt/CARLA/Simulator",
                "SCENARIORUNNER_PATH": "/opt/CARLA/ScenarioRunner",
                "OPENSBT_CORE_PATH": "/opt/OpenSBT/Core",
                "OPENSBT_RUNNER_PATH": "/opt/OpenSBT/Runner",
            }
        }
    ]
}
```
