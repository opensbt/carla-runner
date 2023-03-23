import logging
import traceback

from carla_simulation.balancer import Balancer

b = Balancer(
    directory = '/home/groh/projects/ff1_carla/scenarios2',
    jobs = 1,
    visualization = True
)
try:
    b.start()

    e = b.run()
    print(e)
except Exception as e:
    logging.error(traceback.format_exc())
finally:
    b.stop()
