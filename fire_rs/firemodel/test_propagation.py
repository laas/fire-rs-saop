import unittest
import fire_rs.firemodel.propagation as propagation


class TestPropagation(unittest.TestCase):

    def test_propagate(self):
        env = propagation.Environment([[475060.0, 477060.0], [6200074.0, 6202074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20)
        # prop.plot(blocking=True)

