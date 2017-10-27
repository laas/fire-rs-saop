# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import unittest

import gdal
import numpy as np

from fire_rs.geodata.environment import World, CORINELANDCOVER_TO_FUELMODEL_REMAP


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
        ret = world.get_wind([475060.0, 6200074.0], domain_average=(3.11, np.pi/2))

    def test_get_wind_on_area(self):
        world = World()
        res = world.get_wind([[475060.0, 476160.0], [6200074.0, 6201174.0]], domain_average=(3.11, np.pi))

    def test_get_slope_on_area(self):
        world = World()
        res = world.get_slope([[475060.0, 476060.0], [6200074.0, 6200174.0]])

    def test_get_landcover_class(self):
        world = World()
        res =world.get_landcover_class([475060.0, 6200074.0])

    def test_get_landcover_class_on_area(self):
        world = World()
        res = world.get_landcover_class([[475060.0,485060], [6200074.0, 6210074]])

    def test_get_fuel_type(self):
        world = World()
        res = world.get_fuel_type([475060.0, 6200074.0], CORINELANDCOVER_TO_FUELMODEL_REMAP)

    def test_get_fuel_type_on_area(self):
        world = World()
        res = world.get_fuel_type([[475060.0,485060], [6200074.0, 6210074]], CORINELANDCOVER_TO_FUELMODEL_REMAP)

    def test_get_wind_elevation_on_area(self):
        world = World()
        area = [[475060.0, 476060.0], [6200074.0, 6200174.0]]
        elevation_data = world.get_elevation(area)
        wind_data = world.get_wind(area, domain_average=(3.11, np.pi/2))
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
