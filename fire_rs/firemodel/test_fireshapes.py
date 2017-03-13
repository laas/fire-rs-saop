import unittest
import numpy as np
import itertools
import fire_rs.firemodel.fireshapes as fireshapes


class TestFireShapesConsistency(unittest.TestCase):
    
    def test_decreasing_speed(self):
        """Make sure the rate of spread is strictly decreasing as we move away from the main direction"""
        for target_angle in np.linspace(0.01, np.pi, num=100):
            prev_ros = 999999
            for windspeed in np.linspace(0, 100, num=100):
                shape = fireshapes.get_fire_shape(windspeed, 0, 10)
                ros = shape.speed(target_angle)
                self.assertLess(ros, prev_ros, 'Speed increases when going backward')
                prev_ros = ros

    def test_circular_when_no_wind(self):
        """Check that we get a circular shape when there is no wind"""
        shape = fireshapes.get_fire_shape(0, 0, 10)
        for target_angle in np.linspace(0, 2*np.pi):
            self.assertAlmostEqual(10, shape.speed(target_angle))

    def test_main_ros(self):
        """Make sure the ros in the main direction is the good one."""
        for ros in np.linspace(0.01, 100, num=100):
            for windspeed in np.linspace(0, 100, num=100):
                shape = fireshapes.get_fire_shape(windspeed, 0, ros)
                self.assertAlmostEqual(ros, shape.speed(0))

if __name__ == '__main__':
    unittest.main()
