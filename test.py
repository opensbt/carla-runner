from carla_simulation.balancer import Balancer

b = Balancer(
    directory = '/tmp/scenarios',
    jobs = 1,
    visualization = True
)

b.start()

e = b.run()
print(e)

b.stop()
