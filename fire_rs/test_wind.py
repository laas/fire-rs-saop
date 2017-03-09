import tempfile
import gdal
import numpy as np
import os
import unittest

from wind import WindTile, WindNinjaCLI


class WindNinjaCLITest(unittest.TestCase):

    def setUp(self):
        self.output_path = tempfile.gettempdir()

    def domain_average_run_blocking_test(self):
        cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file('/home/rbailonr/test_31.tif')
        cli.set_output_path(self.output_path)
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(completed.returncode, 0,
                             "WindNinja_cli execution failed! Return code is {} instead of 0.".format(completed.returncode))

    def domain_average_diurnal_winds_test(self):
        cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file('/home/rbailonr/test_31.tif')
        cli.set_output_path(self.output_path)
        cli.add_arguments(diurnal_winds='true',
                          uni_air_temp=20, air_temp_units='C',
                          uni_cloud_cover=0.2, cloud_cover_units='fraction',
                          year=2017, month=3, day=6, hour=11, minute=0, time_zone='Europe/Paris')
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(completed.returncode, 0,
                             "WindNinja_cli execution failed! Return code is {} instead of 0.".format(completed.returncode))


class WindTileTest(unittest.TestCase):

    def setUp(self):
        cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file('/home/rbailonr/test_31.tif')

        self.wind_path = tempfile.gettempdir()
        self.windvel_file = os.path.join(self.wind_path, 'test_31_0_3_100m_vel.asc')
        self.windang_file = os.path.join(self.wind_path, 'test_31_0_3_100m_ang.asc')

        cli.set_output_path(self.wind_path)
        completed = cli.run_blocking()

    def test_access(self):
        tile = WindTile(self.windvel_file, self.windang_file)
        w = tile.get_wind(np.array([512748, 6211766]))
        self.assertEqual(len(w), 2)


if __name__ == '__main__':

    def gdal_error_handler(err_class, err_num, err_msg):
        errtype = {
                gdal.CE_None:'None',
                gdal.CE_Debug:'Debug',
                gdal.CE_Warning:'Warning',
                gdal.CE_Failure:'Failure',
                gdal.CE_Fatal:'Fatal'
        }
        err_msg = err_msg.replace('\n',' ')
        err_class = errtype.get(err_class, 'None')
        print('Error Number: %s' % (err_num))
        print('Error Type: %s' % (err_class))
        print('Error Message: %s' % (err_msg))
    # install error handler
    gdal.PushErrorHandler(gdal_error_handler)
    # Enable GDAL/OGR exceptions
    gdal.UseExceptions()

    unittest.main()
