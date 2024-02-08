import json
import logging
import traceback

from carla_simulation.balancer import Balancer

# Please use "git update-index --assume-unchanged test.py" if you use and
# change this file to disable pushing your local changes for it on the server!

balancer = Balancer(
    # If the path changes or the quality, the container needs to be rebuilt.
    # Delete all client containers for this to happen:
    # `docker container rm <client-container-name>` (e.g., carla-client-0).
    scenarios_dir='/path/to/your/test/scenarios',
    agent='FMIAgent',
    jobs=1,
    visualization=True,
    enable_manual_control = False,
    rendering_quality = "Medium" # Low, Medium, Epic
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
