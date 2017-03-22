import unittest
import fire_rs.firemodel.propagation as propagation


class TestPropagation(unittest.TestCase):

    def test_propagate(self):
        env = propagation.Environment([[475060.0, 476060.0], [6200074.0, 6200174.0]], wind_speed=4.11, wind_dir=90.2)
        ignitions = propagation.propagate(env, 0, 0)
        ignitions.write_to_file('/tmp/igni.tif')
