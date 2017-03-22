
import datetime
import fire_rs.firemodel.propagation as propagation


for side in [10, 20, 40, 50, 60, 80, 100, 120]:
    env = propagation.DummyEnvironment(side, side, 10)
    start = datetime.datetime.now()
    propagation.propagate(env, 5, 5)
    end = datetime.datetime.now()
    elapsed = end - start
    print('On a {} grid: \t{}.{} seconds   ({} usecs / celll)'.format(
        side, elapsed.seconds, elapsed.microseconds, (elapsed / side / side).microseconds))
