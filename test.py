from balancer import Balancer

b = Balancer(
    directory = '/home/munaro/Repositories/ASCRIBE/Simulation/scenarios',
    jobs = 1
)

b.start()

e = None
try:
    e = b.run()
except:
    pass
print(e)

b.stop()

# Reuse
# Visualization
# Parallelization
