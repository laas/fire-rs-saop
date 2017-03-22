import unittest

import gdal
import numpy as np

from fire_rs.geodata.environment import World


class WorldTest(unittest.TestCase):

    def setUp(self):
        pass

    def test_get_elevation(self):
        world = World()
        world.get_elevation([475060.0, 6200074.0])

    def test_get_elevation_on_area(self):
        world = World()
        world.get_elevation([[475060.0,485060], [6200074.0, 6210074]])

    def test_get_wind(self):
        world = World()
        world.get_wind([475060.0, 6200074.0], domain_average=(3.11, 90.2))

    def test_get_wind_on_area(self):
        world = World()
        res = world.get_wind([[475060.0, 476060.0], [6200074.0, 6200174.0]], domain_average=(3.11, 90.2))

    def test_get_slope_on_area(self):
        world = World()
        res = world.get_slope([[475060.0, 476060.0], [6200074.0, 6200174.0]])


    def test_get_wind_elevation_on_area(self):
        world = World()
        area = [[475060.0, 476060.0], [6200074.0, 6200174.0]]
        elevation_data = world.get_elevation(area)
        wind_data = world.get_wind(area, domain_average=(3.11, 90.2))
        self.assertEqual(elevation_data.data.shape, wind_data.data.shape)
        self.assertEqual(elevation_data.x_offset, wind_data.x_offset)
        self.assertEqual(elevation_data.y_offset, wind_data.y_offset)
        self.assertEqual(elevation_data.cell_width, wind_data.cell_width)
        self.assertEqual(elevation_data.cell_height, wind_data.cell_height)

        elev_wind = elevation_data.combine(wind_data)
        self.assertEqual(elevation_data.data.shape, elev_wind.data.shape)
        self.assertEqual(elevation_data.x_offset, elev_wind.x_offset)
        self.assertEqual(elevation_data.y_offset, elev_wind.y_offset)
        self.assertEqual(elevation_data.cell_width, elev_wind.cell_width)
        self.assertEqual(elevation_data.cell_height, elev_wind.cell_height)


if __name__ == '__main__':
    gdal.UseExceptions()
    unittest.main()
