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

if __name__ == '__main__':
    unittest.main()
