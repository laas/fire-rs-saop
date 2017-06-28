import unittest
import fire_rs.firemodel.propagation as propagation


class TestPropagation(unittest.TestCase):


    def setUp(self):
        self.test_area = [[480060.0, 490060.0], [6210074.0, 6220074.0]]

    def test_propagate(self):
        env = propagation.Environment(self.test_area, wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20, horizon=3*3600)
        # prop.plot(blocking=True)

    def test_propagate_full(self):
        env = propagation.Environment(self.test_area, wind_speed=18.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20)
        # prop.plot(blocking=True)
