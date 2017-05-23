import unittest
import fire_rs.uav_planning as up


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.uav = up.UAV(1, 1)
        self.wp1 = up.Waypoint(0, 0, 0)
        self.wp2 = up.Waypoint(4, 0, 0)

    def test_waypoint(self):
        wp1 = up.Waypoint(4, 3, 2)
        self.assertEquals(wp1.x, 4)
        self.assertEquals(wp1.y, 3)
        self.assertEquals(wp1.dir, 2)

    def test_straight_line_dubins(self):
        self.assertAlmostEquals(4, self.uav.travel_distance(self.wp1, self.wp2))
