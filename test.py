import logging
import traceback

from carla_simulation.balancer import Balancer

balancer = Balancer(
    # If the path changes, the container needs to be rebuilt. Delete all client containers for this to happen
    directory='/tmp/scenarios',
    jobs=1,
    visualization=True,
    keep_carla_servers=True
)
try:
    balancer.start()

    evaluations = balancer.run()
    print(evaluations)
except Exception as exception:
    logging.error(traceback.format_exc())
finally:
    balancer.stop()
