import unittest
from wind import WindNinjaCLI


class WindNinjaCLITest(unittest.TestCase):

    def domain_average_run_blocking_test(self):
        cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file('/home/rbailonr/test_31.tif')
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(completed.returncode, 0,
                             "WindNinja_cli execution failed! Return code is {} instead of 0.".format(completed.returncode))

    def domain_average_diurnal_winds_test(self):
        cli = WindNinjaCLI('/home/rbailonr/bin/windninja_cli/')
        cli.add_arguments(**WindNinjaCLI.domain_average_args(3.0, 0))
        cli.set_elevation_file('/home/rbailonr/test_31.tif')
        cli.add_arguments(diurnal_winds='true',
                          uni_air_temp=20, air_temp_units='C',
                          uni_cloud_cover=0.2, cloud_cover_units='fraction',
                          year=2017, month=3, day=6, hour=11, minute=0, time_zone='Europe/Paris')
        completed = cli.run_blocking()
        if completed.returncode != 0:
            self.assertEqual(completed.returncode, 0,
                             "WindNinja_cli execution failed! Return code is {} instead of 0.".format(completed.returncode))

if __name__ == '__main__':
    unittest.main()
