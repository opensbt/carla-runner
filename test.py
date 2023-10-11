import logging
import traceback

from carla_simulation.balancer import Balancer

# Please use "git update-index --assume-unchanged test.py" if you use and
# change this file to disable pushing your local changes for it on the server!

balancer = Balancer(
    # If the path changes or the quality, the container needs to be rebuilt.
    # Delete all client containers for this to happen:
    # `docker container rm <client-container-name>` (e.g., carla-client-0).
    directory='/path/to/your/test/scenarios',
    agent='FMIAgent',
    jobs=1,
    visualization=True,
    keep_carla_servers=False,
    temporal_resolution = 0.1,
    synchronous_execution = True,
    enable_manual_control = False,
    rendering_quality = "Medium" # Low, Medium, Epic
)

try:
    balancer.start()
    evaluations = balancer.run()
    print(evaluations)
except Exception as exception:
    logging.error(traceback.format_exc())
finally:
    balancer.stop()
