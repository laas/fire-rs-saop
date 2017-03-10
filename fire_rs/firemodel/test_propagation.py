import unittest
import fire_rs.firemodel.propagation as propagation


class TestPropagation(unittest.TestCase):

    def test_propagate(self):
        env = propagation.DummyEnvironment(70, 70, 10)
        propagation.propagate(env, 5, 5)
