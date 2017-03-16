import gdal
import numpy as np
import unittest

from fire_rs.geodata.environment import World


class WorldTest(unittest.TestCase):

    def setUp(self):
        pass

    def test_get_elevation(self):
        world = World()
        res = world.get_elevation(np.array([475060.0, 6200074.0]))
        self.assertEqual(len(world.get_elevation([475060.0, 6200074.0])), True)

    def test_get_wind(self):
        world = World()
        world.get_wind(np.array([475060.0, 6200074.0]), domain_average=(3.11, 90.2))

if __name__ == '__main__':
    gdal.UseExceptions()
    unittest.main()
