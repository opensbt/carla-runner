import logging
import traceback

from carla_simulation.balancer import Balancer

balancer = Balancer(
    # If the path changes, the container needs to be rebuilt.
    # Delete all client containers for this to happen (docker container rm <client-container-name>, e.g., carla-client-0).
    directory='/home/sebastian/work_repos/Simulation/Examples/Scenarios',
    agent='FMIAgent',
    jobs=1,
    visualization=True,
    keep_carla_servers=False,
    resolution = 0.1,
    synchronous = True,
    enable_manual_control = True
)

try:
    balancer.start()
    evaluations = balancer.run()
    print(evaluations)
except Exception as exception:
    logging.error(traceback.format_exc())
finally:
    balancer.stop()
