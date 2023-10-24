import json
import logging
import traceback

from carla_simulation.balancer import Balancer

balancer = Balancer(
    # If the path changes, the container needs to be rebuilt.
    # Delete all client containers for this to happen.
    directory='/tmp/scenarios',
    jobs=1,
    visualization=True,
    fault="/tmp/faults",
    keep_carla_servers=False
)

try:
    balancer.start()
    evaluations = balancer.run()

    with open('output.json', 'w') as file:
        json.dump(evaluations, file, indent=2)

except Exception as exception:
    logging.error(traceback.format_exc())
finally:
    balancer.stop()