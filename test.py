import logging
import traceback

from carla_simulation.balancer import Balancer

b = Balancer(
    directory='/tmp/scenarios',
    jobs=1,
    visualization=True,
    keep_carla_servers=True
)
try:
    b.start()

    e = b.run()
    print(e)
except Exception as e:
    logging.error(traceback.format_exc())
finally:
    b.stop()
