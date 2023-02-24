from balancer import Balancer

b = Balancer(
    directory = '/tmp/scenarios',
    jobs = 1,
    visualization = True
)

b.start()

e = None
try:
    e = b.run()
except:
    pass
print(e)

b.stop()
