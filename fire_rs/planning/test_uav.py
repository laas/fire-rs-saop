import unittest

from fire_rs.planning.uav import UAV


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.uav = UAV(10, 20, 2, 3)

    def test_uav_speed(self):
        self.assertAlmostEquals(5, self.uav.travel_time((0, 0), (100, 0)))
        self.assertAlmostEquals(5, self.uav.travel_time((0, 0), (0, 100)))
        self.assertAlmostEquals(5, self.uav.travel_time((0, 0, 0), (0, 100, 1)))
        self.assertAlmostEquals(5, self.uav.travel_time((0, 0, 4), (0, 100, 1)))

    def test_uav_climb(self):
        self.assertAlmostEquals(1, self.uav.travel_time((0, 0, 0), (0, 0, 2)))
        self.assertAlmostEquals(45, self.uav.travel_time((0, 0, 0), (0, 100, 90)))

    def test_uav_descent(self):
        self.assertAlmostEquals(1, self.uav.travel_time((0, 0, 100), (0, 0, 97)))
        self.assertAlmostEquals(30, self.uav.travel_time((0, 0, 90), (0, 100, 0)))