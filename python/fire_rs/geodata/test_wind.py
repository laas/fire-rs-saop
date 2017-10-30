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

import os
import tempfile
import unittest
from datetime import datetime

import gdal
import numpy as np
from pytz import timezone

from fire_rs.geodata.elevation import ElevationMap, ElevationTile
from fire_rs.geodata.wind import *
from fire_rs.geodata.environment import DEFAULT_FIRERS_DEM_DATA


class WindNinjaCLITest(unittest.TestCase):

    def setUp(self):
        self.output_path = tempfile.gettempdir()

    def test_domain_average_run_blocking(self):
        cli = WindNinjaCLI()
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.add_arguments(mesh_resolution=100)
        cli.set_elevation_file(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                            'BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.tif'))
        cli.set_output_path(self.output_path)
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(
                completed.returncode, 0,
                "WindNinja_cli execution failed! Return code is {} instead of 0.".format(
                    completed.returncode))

    def test_domain_average_diurnal_winds(self):
        cli = WindNinjaCLI()
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                            'BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.tif'))
        cli.set_output_path(self.output_path)
        cli.add_arguments(mesh_resolution=100, diurnal_winds='true',
                          uni_air_temp=20, air_temp_units='C',
                          uni_cloud_cover=0.2, cloud_cover_units='fraction',
                          year=2017, month=3, day=6, hour=11, minute=0, time_zone='Europe/Paris')
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(
                completed.returncode, 0,
                "WindNinja_cli execution failed! Return code is {} instead of 0.".format(
                    completed.returncode))


class WindMapTest(unittest.TestCase):

    def setUp(self):
        self.output_path = tempfile.gettempdir()

        tile0 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                           'BDALTIV2_25M_FXX_0475_6200_MNT_LAMB93_IGN69.tif'))
        tile1 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                           'BDALTIV2_25M_FXX_0475_6225_MNT_LAMB93_IGN69.tif'))
        tile2 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                           'BDALTIV2_25M_FXX_0500_6200_MNT_LAMB93_IGN69.tif'))
        tile3 = ElevationTile(os.path.join(DEFAULT_FIRERS_DEM_DATA,
                                           'BDALTIV2_25M_FXX_0500_6225_MNT_LAMB93_IGN69.tif'))
        self.elevation_map = ElevationMap([tile0, tile1, tile2, tile3])

        self.cli = WindNinjaCLI()
        self.cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 3*np.pi/4))
        self.cli.add_arguments(**WindNinjaCLI.output_type_args(True, False, True,
                                                               False, False, False))
        self.cli.add_arguments(**WindNinjaCLI.diurnal_winds_args(
            20, 0.25, datetime(2017, 11, 15, 12, 30, 0, 0, timezone('Europe/Paris'))))
        self.cli.add_arguments(mesh_resolution=100)
        self.cli.set_output_path(self.output_path)

    def test_access(self):
        wmap = WindMap([], self.elevation_map, self.cli)
        w = wmap.get_wind(np.array([512748, 6211766]))
        self.assertEqual(len(w), 2)


class WindAngleTransformTest(unittest.TestCase):

    def test_geo_to_trigo(self):
        self.assertAlmostEqual(0, geo_angle_to_trigo_angle(90))
        self.assertAlmostEqual(np.pi/2 % (np.pi * 2), geo_angle_to_trigo_angle(180))
        self.assertAlmostEqual(-np.pi/2 % (np.pi * 2), geo_angle_to_trigo_angle(0))
        self.assertAlmostEqual(np.pi, geo_angle_to_trigo_angle(270))

    def test_trigo_to_geo(self):
        self.assertAlmostEqual(90, trigo_angle_to_geo_angle(0))
        self.assertAlmostEqual(0, trigo_angle_to_geo_angle(-np.pi/2))
        self.assertAlmostEqual(180, trigo_angle_to_geo_angle(np.pi/2))
        self.assertAlmostEqual(270, trigo_angle_to_geo_angle(np.pi))

    def test_geo_geo(self):
        for a in np.linspace(-1000, 1000):
            self.assertAlmostEqual(a % 360, trigo_angle_to_geo_angle(geo_angle_to_trigo_angle(a)))

    def test_trigo_to_trigo(self):
        for a in np.linspace(-np.pi*5, np.pi*5):
            self.assertAlmostEqual(a % (np.pi*2), geo_angle_to_trigo_angle(trigo_angle_to_geo_angle(a)))


if __name__ == '__main__':

    def gdal_error_handler(err_class, err_num, err_msg):
        errtype = {
            gdal.CE_None: 'None',
            gdal.CE_Debug: 'Debug',
            gdal.CE_Warning: 'Warning',
            gdal.CE_Failure: 'Failure',
            gdal.CE_Fatal: 'Fatal'
        }
        err_msg = err_msg.replace('\n', ' ')
        err_class = errtype.get(err_class, 'None')
        print('Error Number: %s' % (err_num))
        print('Error Type: %s' % (err_class))
        print('Error Message: %s' % (err_msg))
    # install error handler
    gdal.PushErrorHandler(gdal_error_handler)
    # Enable GDAL/OGR exceptions
    gdal.UseExceptions()

    unittest.main()
