import unittest

from fire_rs.planning.uav import UAV


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.uav = UAV(10, 20, 2, 3)

    def test_uav_speed(self):
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0), (100, 0)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0), (0, 100)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0, 0), (0, 100, 1)))
        self.assertAlmostEqual(5, self.uav.travel_time((0, 0, 4), (0, 100, 1)))

    def test_uav_climb(self):
        self.assertAlmostEqual(1, self.uav.travel_time((0, 0, 0), (0, 0, 2)))
        self.assertAlmostEqual(45, self.uav.travel_time((0, 0, 0), (0, 100, 90)))

    def test_uav_descent(self):
        self.assertAlmostEqual(1, self.uav.travel_time((0, 0, 100), (0, 0, 97)))
        self.assertAlmostEqual(30, self.uav.travel_time((0, 0, 90), (0, 100, 0)))